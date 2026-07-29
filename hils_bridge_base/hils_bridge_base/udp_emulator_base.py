"""
Base class for HILS UDP device emulator nodes.

Provides shared infrastructure for nodes that emulate Ethernet-connected
devices (LiDAR, industrial cameras, etc.) via USB-Ethernet adapters.

Subclasses implement the device-specific protocol; this base handles:
  - Network interface detection and IP validation
  - UDP socket creation with interface binding (SO_BINDTODEVICE)
  - Rate limiting
  - Subnet validation between emulator IP and host IP
  - Parameter declaration (device_ip, host_ip, network_interface, max_hz)
  - Periodic statistics logging

Usage:
    class VelodyneEmulatorNode(UdpEmulatorBase):
        def __init__(self):
            super().__init__(
                node_name='hils_velodyne_emulator',
                default_device_ip='192.168.1.201',
                default_host_ip='192.168.1.100',
            )
            # Create device-specific sockets
            self._data_sock = self.create_device_socket(2368)
            # Subscribe to ROS topics
            self.create_subscription(PointCloud2, '/points', self.pc_callback, qos)
"""

import functools
import socket
import time
import threading

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from hils_bridge_base import network_utils
from hils_bridge_base.fault_injection import DelayedSender, FaultPipeline


class UdpEmulatorBase(Node):
    """Base class for USB-Ethernet adapter + software UDP emulator nodes."""

    def __init__(self, node_name: str, *,
                 default_device_ip: str = '192.168.1.12',
                 default_host_ip: str = '192.168.1.5'):
        super().__init__(node_name)

        # Parameters
        self.declare_parameter('device_ip', default_device_ip,
            ParameterDescriptor(
                description='IP address assigned to the USB-Ethernet adapter '
                            '(emulates the real device IP).'))
        self.declare_parameter('host_ip', default_host_ip,
            ParameterDescriptor(
                description='IP address of the robot PC that receives packets.'))
        self.declare_parameter('network_interface', '',
            ParameterDescriptor(
                description='Network interface to bind (e.g. "eth1"). '
                            'Empty = auto-detect from device_ip.'))
        self.declare_parameter('max_hz', 10.0,
            ParameterDescriptor(
                description='Maximum output rate in Hz.'))

        self.add_on_set_parameters_callback(self._on_param_change)

        self._host_ip = self.get_parameter('host_ip').value
        self._network_interface = self.get_parameter('network_interface').value

        # Resolve device IP from interface or parameter
        if self._network_interface:
            self._device_ip, netmask = network_utils.get_interface_ip_and_netmask(
                self._network_interface)
            self.get_logger().info(
                f'Using {self._network_interface}: ip={self._device_ip}, '
                f'netmask={netmask}')
        else:
            self._device_ip = self.get_parameter('device_ip').value
            if not network_utils.verify_ip_available(self._device_ip):
                self.get_logger().error(
                    f'{self._device_ip} is not assigned to any interface. '
                    f'Set network_interface parameter, or manually run:\n'
                    f'  sudo ip addr add {self._device_ip}/24 dev <interface>')
                raise RuntimeError(f'Cannot bind to {self._device_ip}')
            netmask = network_utils.find_netmask_for_ip(self._device_ip)

        # Validate subnet
        if netmask:
            if not network_utils.validate_subnet(
                    self._device_ip, self._host_ip, netmask):
                raise RuntimeError(
                    f'{self._host_ip} is not on the same subnet as '
                    f'{self._device_ip}')
            self.get_logger().info(
                f'Subnet OK: {self._device_ip} and {self._host_ip} '
                f'are on the same network')

        # State
        self._sockets = []  # track for cleanup
        self._lock = threading.Lock()
        self._last_send_time = 0.0
        self._send_count = 0
        self._udp_cnt = 0

        # Fault injection: with no active faults, send_udp() is a direct
        # sendto() and behavior is identical to the pre-fault-injection
        # implementation.
        self._fault_pipeline = FaultPipeline()
        self._delayed_sender = DelayedSender(
            on_error=lambda e: self.get_logger().error(
                f'Delayed send failed: {e}'))
        self._fault_controller = None
        self.declare_parameter('enable_fault_services', True,
            ParameterDescriptor(
                description='Create ~/inject_fault, ~/clear_fault and '
                            '~/get_fault_state services.'))
        if self.get_parameter('enable_fault_services').value:
            try:
                from hils_bridge_base.fault_injection.fault_controller \
                    import FaultInjectionController
                self._fault_controller = FaultInjectionController(
                    self, self._fault_pipeline, self._delayed_sender)
            except ImportError as e:
                self.get_logger().warning(
                    f'Fault injection services unavailable '
                    f'(hils_bridge_interfaces not built?): {e}')

        # Stats timer
        self.create_timer(5.0, self._stats_callback)

    def _on_param_change(self, params):
        for param in params:
            if param.name in ('max_hz',):
                self.get_logger().info(f'{param.name} changed to {param.value}')
        return SetParametersResult(successful=True)

    @property
    def device_ip(self) -> str:
        """The resolved IP address of the emulated device."""
        return self._device_ip

    @property
    def host_ip(self) -> str:
        """The IP address of the robot PC."""
        return self._host_ip

    def create_device_socket(self, port: int, *,
                             reuse: bool = False,
                             bind_any: bool = False) -> socket.socket:
        """Create a UDP socket bound to the device IP and port.

        Args:
            port: Port number to bind.
            reuse: Set SO_REUSEADDR.
            bind_any: Bind to INADDR_ANY instead of device_ip
                      (useful for receiving broadcasts).

        Returns:
            Configured non-blocking UDP socket.
        """
        bind_ip = '' if bind_any else self._device_ip
        sock = network_utils.create_udp_socket(
            bind_ip, port,
            network_interface=self._network_interface,
            reuse=reuse)
        self._sockets.append(sock)
        return sock

    @property
    def fault_pipeline(self) -> FaultPipeline:
        """The fault injection pipeline applied by send_udp()."""
        return self._fault_pipeline

    def send_udp(self, sock: socket.socket, data: bytes, addr,
                 *, channel: str = 'data') -> bool:
        """Send a UDP packet through the fault injection pipeline.

        With no active faults this is a plain sock.sendto(). Active
        faults may drop the packet (returns True: the drop is
        intentional), corrupt it, duplicate it, or defer it via the
        delayed sender thread.

        Args:
            sock: Socket to send on.
            data: UDP payload.
            addr: (host, port) destination.
            channel: Logical stream name faults can target
                     (e.g. 'data', 'position', 'imu').

        Returns:
            True unless an immediate send failed with OSError.
        """
        for pkt in self._fault_pipeline.apply(bytes(data), channel):
            if pkt.delay_s <= 0.0:
                try:
                    sock.sendto(pkt.data, addr)
                except OSError as e:
                    self.get_logger().error(f'UDP send failed: {e}')
                    return False
            else:
                self._delayed_sender.submit(
                    pkt.delay_s, functools.partial(sock.sendto, pkt.data, addr))
        return True

    def check_rate_limit(self) -> bool:
        """Check if enough time has passed since the last send.

        Returns:
            True if the message should be sent, False if rate-limited.
        """
        now = time.monotonic()
        min_interval = 1.0 / self.get_parameter('max_hz').value
        if (now - self._last_send_time) < min_interval:
            return False
        return True

    def mark_sent(self):
        """Update timestamp and counter after a successful send."""
        self._last_send_time = time.monotonic()
        self._send_count += 1

    def next_udp_cnt(self) -> int:
        """Return and increment the UDP packet counter."""
        cnt = self._udp_cnt
        self._udp_cnt += 1
        return cnt

    def _stats_callback(self):
        self.get_logger().info(
            f'Status: sent={self._send_count}, '
            f'device_ip={self._device_ip}')

    def destroy_node(self):
        # Fail-safe (docs section 17.5): discard queued delayed packets
        # instead of flushing stale data after shutdown.
        if hasattr(self, '_delayed_sender'):
            discarded = self._delayed_sender.stop()
            if discarded:
                self.get_logger().info(
                    f'Discarded {discarded} pending delayed packet(s)')
        for sock in self._sockets:
            try:
                sock.close()
            except Exception:
                pass
        super().destroy_node()
