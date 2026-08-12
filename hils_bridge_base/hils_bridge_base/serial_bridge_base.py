"""
Base class for HILS serial bridge nodes (FT234X cross-connection).

Provides shared infrastructure for nodes that:
  1. Subscribe to a ROS topic
  2. Convert data to a device-specific serial protocol (NMEA, binary, etc.)
  3. Write the protocol data to a serial port (FT234X on /dev/ttyUSBx)

Subclasses implement the protocol conversion; this base handles:
  - Serial port open/close and error recovery
  - Rate limiting
  - Parameter declaration (serial_port, baudrate, max_hz)
  - Periodic statistics logging

Usage:
    class GpsBridgeNode(SerialBridgeBase):
        def __init__(self):
            super().__init__(
                node_name='hils_gps_bridge',
                default_baudrate=9600,
            )
            self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)

        def gps_callback(self, msg):
            nmea = self._navsatfix_to_nmea(msg)
            self.serial_write(nmea.encode())
"""

import functools
import os
import time
import threading
import tty

import serial

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from hils_bridge_base import frame_protocol
from hils_bridge_base.device_state import DeviceStateMachine
from hils_bridge_base.device_state import state as device_states
from hils_bridge_base.fault_injection import DelayedSender, FaultPipeline


class _PtyTransport:
    """os.openpty()-backed drop-in for serial.Serial (SILS, no adapter).

    Presents the same surface the bridge nodes use on a serial.Serial
    (write/flush/read/close/is_open/baudrate): the node talks to the
    PTY master while the peer program opens a symlink to the slave pts,
    exactly how REACT-simulator's hardware_emulator VirtualSerialPort
    presents virtual devices. The slave fd is kept open on our side so
    reads on the master see EAGAIN (no data) instead of EIO when the
    peer closes and reopens the port.
    """

    def __init__(self, link_path: str, baudrate: int = 0):
        self._master_fd, self._slave_fd = os.openpty()
        # Raw mode: no echo / line discipline mangling of binary protocols.
        tty.setraw(self._slave_fd)
        os.set_blocking(self._master_fd, False)

        # Replace whatever sits at link_path with a symlink to the pts.
        slave_path = os.ttyname(self._slave_fd)
        if os.path.islink(link_path) or os.path.exists(link_path):
            os.remove(link_path)
        os.symlink(slave_path, link_path)
        self._link_path = link_path

        # Meaningless on a PTY; kept assignable so the baudrate
        # param-change handler in SerialBridgeBase stays a safe no-op.
        self.baudrate = baudrate
        self.is_open = True

    @property
    def link_path(self) -> str:
        """The symlink path the peer program opens."""
        return self._link_path

    def write(self, data: bytes) -> int:
        try:
            return os.write(self._master_fd, data)
        except BlockingIOError:
            # PTY buffer full (peer not draining): drop, like an
            # unread UART FIFO, rather than blocking the executor.
            return 0
        except OSError as e:
            raise serial.SerialException(f'PTY write failed: {e}') from e

    def flush(self) -> None:
        """No-op: os.write() to the master has no userspace buffer."""

    def read(self, n: int) -> bytes:
        """Non-blocking read; b'' when no data is available."""
        try:
            return os.read(self._master_fd, n)
        except (BlockingIOError, OSError):
            return b''

    def close(self) -> None:
        if not self.is_open:
            return
        self.is_open = False
        for fd in (self._master_fd, self._slave_fd):
            try:
                os.close(fd)
            except OSError:
                pass
        # Remove the symlink so a stale path never points at a dead pts.
        try:
            if os.path.islink(self._link_path):
                os.remove(self._link_path)
        except OSError:
            pass


class SerialBridgeBase(Node):
    """Base class for FT234X cross-connection serial bridge nodes."""

    def __init__(self, node_name: str, *, default_baudrate: int = 115200,
                 default_serial_port: str = '/dev/ttyUSB0',
                 default_max_hz: float = 10.0,
                 frame_protocol_firmware: bool = False):
        """frame_protocol_firmware: the peer is a HILS firmware speaking
        the AA55 frame protocol; enables firmware-cooperative faults
        (docs 9.3). Leave False for raw-protocol peers (NMEA, WT901)
        where 0x50-series frames would just pollute the byte stream.
        """
        super().__init__(node_name)

        # Parameters
        self.declare_parameter('serial_port', default_serial_port)
        self.declare_parameter('baudrate', default_baudrate,
            ParameterDescriptor(
                description='Serial baudrate. Must match the real sensor driver expectation.'))
        self.declare_parameter('max_hz', default_max_hz,
            ParameterDescriptor(
                description='Maximum output rate in Hz.'))
        # A peer that stops reading (e.g. wrong firmware on a USB CDC
        # device) would otherwise block write() forever inside the
        # executor and silently freeze the whole node.
        self.declare_parameter('write_timeout_sec', 1.0,
            ParameterDescriptor(
                description='Serial write timeout. A timed-out write is '
                            'logged and dropped instead of blocking the '
                            'node forever.'))
        # SILS without any USB-serial hardware: instead of opening
        # serial_port as an existing device, create a PTY pair and
        # symlink serial_port -> slave pts. The peer program opens the
        # symlink as if it were the real device.
        self.declare_parameter('create_pty', False,
            ParameterDescriptor(
                description='Create a virtual serial device (PTY) and '
                            'symlink serial_port to it instead of '
                            'opening serial_port as an existing device.'))

        self.add_on_set_parameters_callback(self._on_param_change)

        # Open serial port (or create the virtual one)
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        if self.get_parameter('create_pty').value:
            try:
                self._serial = _PtyTransport(port, baudrate)
                self.get_logger().info(
                    f'Created virtual serial device (PTY): {port}')
            except OSError as e:
                self.get_logger().error(f'Failed to create PTY {port}: {e}')
                raise
        else:
            try:
                self._serial = serial.Serial(
                    port, baudrate, timeout=0,
                    write_timeout=self.get_parameter('write_timeout_sec').value)
                self.get_logger().info(f'Opened serial port: {port} @ {baudrate} baud')
            except serial.SerialException as e:
                self.get_logger().error(f'Failed to open serial port {port}: {e}')
                raise

        self._serial_lock = threading.Lock()
        self._last_send_time = 0.0
        self._send_count = 0

        # Device state machine (docs section 4.2). Initial state
        # STREAMING keeps normal operation unchanged.
        self.declare_parameter('initial_device_state',
                               device_states.STREAMING,
            ParameterDescriptor(
                description='Device state at startup (docs section 4.2).'))
        self._device_state = DeviceStateMachine(
            self.get_parameter('initial_device_state').value)
        self._state_controller = None

        # Fault injection: with no active faults, serial_write() behaves
        # exactly as the pre-fault-injection implementation.
        self._fault_pipeline = FaultPipeline()
        # Firmware-cooperative faults (docs 9.3): commands are framed
        # and written directly, bypassing the pipeline so an active
        # drop fault cannot swallow its own clear command.
        if frame_protocol_firmware:
            self._fault_pipeline.set_firmware_transport(
                lambda payload: self._raw_serial_write(
                    frame_protocol.build_frame(payload)))
        self._delayed_sender = DelayedSender(
            on_error=lambda e: self.get_logger().error(
                f'Delayed serial write failed: {e}'))
        self._fault_controller = None
        self.declare_parameter('enable_fault_services', True,
            ParameterDescriptor(
                description='Create ~/inject_fault, ~/clear_fault and '
                            '~/get_fault_state services.'))
        if self.get_parameter('enable_fault_services').value:
            try:
                from hils_bridge_base.fault_injection.fault_controller \
                    import FaultInjectionController
                from hils_bridge_base.device_state.state_controller \
                    import DeviceStateController
                self._fault_controller = FaultInjectionController(
                    self, self._fault_pipeline, self._delayed_sender)
                self._state_controller = DeviceStateController(
                    self, self._device_state)
            except ImportError as e:
                self.get_logger().warning(
                    f'Fault injection services unavailable '
                    f'(hils_bridge_interfaces not built?): {e}')

        # Stats timer
        self.create_timer(10.0, self._stats_callback)

    def _on_param_change(self, params):
        for param in params:
            if param.name == 'baudrate':
                # On a real port pyserial reconfigures the line; on a
                # _PtyTransport this is a plain attribute store (a PTY
                # has no baudrate), so the change is a safe no-op.
                with self._serial_lock:
                    self._serial.baudrate = param.value
                self.get_logger().info(f'Baudrate changed to {param.value}')
            elif param.name in ('max_hz',):
                self.get_logger().info(f'{param.name} changed to {param.value}')
        return SetParametersResult(successful=True)

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

    @property
    def fault_pipeline(self) -> FaultPipeline:
        """The fault injection pipeline applied by serial_write()."""
        return self._fault_pipeline

    @property
    def device_state(self) -> DeviceStateMachine:
        """The device state machine gating serial_write()."""
        return self._device_state

    def serial_write(self, data: bytes, *, channel: str = 'serial') -> bool:
        """Write data through the state gate and fault pipeline.

        The device state machine may suppress the channel entirely
        (e.g. power_off, rebooting; returns True: the silence is
        intentional). With no active faults this is otherwise a direct
        write. Active faults may drop the data, corrupt it, duplicate
        it, or defer it via the delayed sender thread.

        Returns:
            True unless an immediate write failed.
        """
        if not self._device_state.allows(channel):
            return True
        ok = True
        for pkt in self._fault_pipeline.apply(bytes(data), channel):
            if pkt.delay_s <= 0.0:
                ok = self._raw_serial_write(pkt.data) and ok
            else:
                self._delayed_sender.submit(
                    pkt.delay_s,
                    functools.partial(self._raw_serial_write, pkt.data))
        return ok

    def _raw_serial_write(self, data: bytes) -> bool:
        """Write data to the serial port (thread-safe, no fault pipeline).

        Also updates rate-limit timestamp and send counter.

        Returns:
            True on success, False on error.
        """
        try:
            with self._serial_lock:
                self._serial.write(data)
                self._serial.flush()
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')
            return False

        self._last_send_time = time.monotonic()
        self._send_count += 1
        return True

    def _stats_callback(self):
        self.get_logger().info(
            f'Status: sent={self._send_count}, '
            f'port={self.get_parameter("serial_port").value}')

    def destroy_node(self):
        # Fail-safe (docs section 17.5): discard queued delayed writes
        # instead of flushing stale data after shutdown.
        if hasattr(self, '_delayed_sender'):
            discarded = self._delayed_sender.stop()
            if discarded:
                self.get_logger().info(
                    f'Discarded {discarded} pending delayed write(s)')
        # In create_pty mode _PtyTransport.close() also removes the
        # serial_port symlink so no stale path survives the node.
        if hasattr(self, '_serial') and self._serial.is_open:
            self._serial.close()
        super().destroy_node()
