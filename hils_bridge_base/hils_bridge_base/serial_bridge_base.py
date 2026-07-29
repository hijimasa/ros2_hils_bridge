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
import time
import threading

import serial

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from hils_bridge_base.fault_injection import DelayedSender, FaultPipeline


class SerialBridgeBase(Node):
    """Base class for FT234X cross-connection serial bridge nodes."""

    def __init__(self, node_name: str, *, default_baudrate: int = 115200):
        super().__init__(node_name)

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', default_baudrate,
            ParameterDescriptor(
                description='Serial baudrate. Must match the real sensor driver expectation.'))
        self.declare_parameter('max_hz', 10.0,
            ParameterDescriptor(
                description='Maximum output rate in Hz.'))

        self.add_on_set_parameters_callback(self._on_param_change)

        # Open serial port
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        try:
            self._serial = serial.Serial(port, baudrate, timeout=0)
            self.get_logger().info(f'Opened serial port: {port} @ {baudrate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise

        self._serial_lock = threading.Lock()
        self._last_send_time = 0.0
        self._send_count = 0

        # Fault injection: with no active faults, serial_write() behaves
        # exactly as the pre-fault-injection implementation.
        self._fault_pipeline = FaultPipeline()
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
                self._fault_controller = FaultInjectionController(
                    self, self._fault_pipeline, self._delayed_sender)
            except ImportError as e:
                self.get_logger().warning(
                    f'Fault injection services unavailable '
                    f'(hils_bridge_interfaces not built?): {e}')

        # Stats timer
        self.create_timer(10.0, self._stats_callback)

    def _on_param_change(self, params):
        for param in params:
            if param.name == 'baudrate':
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

    def serial_write(self, data: bytes, *, channel: str = 'serial') -> bool:
        """Write data to the serial port through the fault pipeline.

        With no active faults this is a direct write. Active faults may
        drop the data (returns True: the drop is intentional), corrupt
        it, duplicate it, or defer it via the delayed sender thread.

        Returns:
            True unless an immediate write failed.
        """
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
        if hasattr(self, '_serial') and self._serial.is_open:
            self._serial.close()
        super().destroy_node()
