"""Firmware-cooperative faults (docs section 9.3 / Phase 5).

Some faults cannot be produced by mutating the host-side byte stream:
a USB detach, a lost or truncated UVC frame happen inside the device
firmware. These Fault classes forward the request to the firmware over
the existing frame protocol (0x50-series messages, see
firmware/common/include/hils_frame_protocol.h) and otherwise pass
packets through unchanged.

Lifecycle: FaultPipeline calls on_added()/on_removed() when the fault
is armed/cleared; the pipeline's firmware transport (installed by
SerialBridgeBase) frames and writes the command bytes. The transport
deliberately bypasses the fault pipeline itself so an active drop
fault cannot swallow its own clear command. Constructing these faults
never touches hardware, so the scenario runner can validate specs for
them without a device attached.

They live in hils_bridge_base (not the device package) because the
scenario runner process must be able to import every fault_type
referenced from a scenario file.
"""

from typing import List, Optional

from hils_bridge_base import frame_protocol

from .fault_base import (
    Fault, ScheduledPacket, require_int, reject_unknown_keys,
)

FAULT_CODE_UVC_USB_DETACH = 0x01
FAULT_CODE_UVC_FRAME_DROP = 0x02
FAULT_CODE_UVC_PARTIAL_FRAME = 0x03

# I2C sensor emulator codes (separate namespace from the UVC bridge)
FAULT_CODE_I2C_NACK = 0x01
FAULT_CODE_I2C_RESP_DELAY = 0x02
FAULT_CODE_I2C_REG_FREEZE = 0x03
FAULT_CODE_I2C_WHO_AM_I = 0x04


class FirmwareFault(Fault):
    """Base for faults executed by device firmware.

    Subclasses set fault_code and implement args() -> (arg0, arg1).
    """

    fault_code = 0

    def args(self):
        return (0, 0)

    def process(self, packets: List[ScheduledPacket],
                channel: str) -> List[ScheduledPacket]:
        self.processed_count += 1
        return packets

    def on_added(self, firmware_transport) -> None:
        if firmware_transport is None:
            raise ValueError(
                f'{self.fault_type} needs a firmware transport; this '
                f'node has no device firmware attached')
        arg0, arg1 = self.args()
        self.applied_count += 1
        firmware_transport(
            frame_protocol.build_fault_set(self.fault_code, arg0, arg1))

    def on_removed(self, firmware_transport) -> None:
        if firmware_transport is not None:
            firmware_transport(
                frame_protocol.build_fault_clear(self.fault_code))


class UvcUsbDetachFault(FirmwareFault):
    """Soft USB detach of the UVC device (tud_disconnect on Pico#2).

    Parameters:
        reconnect_after_ms: firmware-side auto-reconnect delay.
            0 (default) = stay detached until the fault is cleared.
    """

    fault_type = 'uvc_usb_detach'
    fault_code = FAULT_CODE_UVC_USB_DETACH

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(params, ('reconnect_after_ms',))
        self.reconnect_after_ms = require_int(
            params, 'reconnect_after_ms', 0, 0, 600000)

    def parameters(self) -> dict:
        return {'reconnect_after_ms': self.reconnect_after_ms}

    def args(self):
        return (self.reconnect_after_ms, 0)


class UvcFrameDropFault(FirmwareFault):
    """Drop received video frames inside the UVC firmware.

    Unlike the host-side drop fault (which stops UART traffic), this
    reproduces bandwidth-starvation frame loss at the device: frames
    arrive intact but are never handed to the UVC stack.

    Parameters:
        percent: fraction of frames to drop, 1-100 (default 100).
            Thinning is a deterministic accumulator, not RNG.
    """

    fault_type = 'uvc_frame_drop'
    fault_code = FAULT_CODE_UVC_FRAME_DROP

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(params, ('percent',))
        self.percent = require_int(params, 'percent', 100, 1, 100)

    def parameters(self) -> dict:
        return {'percent': self.percent}

    def args(self):
        return (self.percent, 0)


class I2cNackFault(FirmwareFault):
    """Make the I2C slave stop ACKing its address (device vanishes).

    The emulator disables its I2C peripheral, so the master sees an
    address NACK on every transaction - the bus-level footprint of a
    disconnected or dead sensor. No parameters.
    """

    fault_type = 'i2c_nack'
    fault_code = FAULT_CODE_I2C_NACK

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        reject_unknown_keys(parameters or {}, ())


class I2cResponseDelayFault(FirmwareFault):
    """Clock-stretch the first read byte of every I2C transaction.

    Parameters:
        delay_us: stretch duration in microseconds, 1-50000
            (default 15000 - beyond most drivers' timeout).
    """

    fault_type = 'i2c_response_delay'
    fault_code = FAULT_CODE_I2C_RESP_DELAY

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(params, ('delay_us',))
        self.delay_us = require_int(params, 'delay_us', 15000, 1, 50000)

    def parameters(self) -> dict:
        return {'delay_us': self.delay_us}

    def args(self):
        return (self.delay_us, 0)


class I2cRegisterFreezeFault(FirmwareFault):
    """Stop applying simulation updates to the register map.

    The master keeps reading successfully but the values never change -
    the footprint of a sensor whose internal sampling died. No
    parameters.
    """

    fault_type = 'i2c_reg_freeze'
    fault_code = FAULT_CODE_I2C_REG_FREEZE

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        reject_unknown_keys(parameters or {}, ())


class I2cWhoAmIFault(FirmwareFault):
    """Serve a wrong WHO_AM_I value (identification mismatch).

    Parameters:
        value: byte returned instead of 0x68 (default 0x00).
    """

    fault_type = 'i2c_who_am_i'
    fault_code = FAULT_CODE_I2C_WHO_AM_I

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(params, ('value',))
        self.value = require_int(params, 'value', 0x00, 0, 255)

    def parameters(self) -> dict:
        return {'value': self.value}

    def args(self):
        return (self.value, 0)


class UvcPartialFrameFault(FirmwareFault):
    """Truncate each UVC frame before transmission (incomplete frame).

    The truncation happens after Pico#1's frame checksum check, so the
    broken JPEG actually reaches the UVC host - unlike the host-side
    corrupt fault, which the inter-Pico protocol filters out.

    Parameters:
        keep_percent: percentage of each frame to transmit, 1-99
            (default 50).
    """

    fault_type = 'uvc_partial_frame'
    fault_code = FAULT_CODE_UVC_PARTIAL_FRAME

    def __init__(self, fault_id: str, *, target: Optional[str] = None,
                 seed: int = 0, parameters: Optional[dict] = None):
        super().__init__(fault_id, target=target, seed=seed)
        params = parameters or {}
        reject_unknown_keys(params, ('keep_percent',))
        self.keep_percent = require_int(params, 'keep_percent', 50, 1, 99)

    def parameters(self) -> dict:
        return {'keep_percent': self.keep_percent}

    def args(self):
        return (self.keep_percent, 0)
