"""Unit tests for firmware-cooperative faults (docs 9.3 / Phase 5).

Run without ROS:
    cd hils_bridge_base && python3 -m pytest test/ -v
"""

import struct

import pytest

from hils_bridge_base import frame_protocol
from hils_bridge_base.fault_injection import (
    FaultPipeline, FaultSpecError, I2cNackFault, I2cResponseDelayFault,
    I2cWhoAmIFault, ScheduledPacket, UvcFrameDropFault,
    UvcPartialFrameFault, UvcUsbDetachFault, create_fault,
)


class FakeTransport:
    def __init__(self):
        self.sent = []

    def __call__(self, payload: bytes):
        self.sent.append(bytes(payload))


def test_payload_layout_matches_c_structs():
    assert frame_protocol.build_fault_set(0x01, 3000, 0) == \
        struct.pack('<BBLL', 0x50, 0x01, 3000, 0)
    assert frame_protocol.build_fault_clear(0x02) == bytes([0x51, 0x02])
    assert frame_protocol.build_reset_bootsel() == \
        struct.pack('<BL', 0x5F, 0x544F4F42)


def test_fault_ack_parse():
    assert frame_protocol.parse_fault_ack(bytes([0x52, 0x01, 0x00])) == \
        (0x01, 'ok')
    assert frame_protocol.parse_fault_ack(bytes([0x52, 0x07, 0x01])) == \
        (0x07, 'unknown_code')
    assert frame_protocol.parse_fault_ack(b'\xff\xd8\xff') is None
    assert frame_protocol.parse_fault_ack(bytes([0x52])) is None


def test_set_and_clear_sent_through_pipeline_hooks():
    transport = FakeTransport()
    pipeline = FaultPipeline()
    pipeline.set_firmware_transport(transport)

    fault = UvcUsbDetachFault('f1', parameters={'reconnect_after_ms': 2500})
    pipeline.add_fault(fault)
    assert transport.sent == [
        frame_protocol.build_fault_set(0x01, 2500, 0)]

    assert pipeline.remove_fault('f1')
    assert transport.sent[-1] == frame_protocol.build_fault_clear(0x01)


def test_clear_all_notifies_every_firmware_fault():
    transport = FakeTransport()
    pipeline = FaultPipeline()
    pipeline.set_firmware_transport(transport)
    pipeline.add_fault(UvcFrameDropFault('d', parameters={'percent': 30}))
    pipeline.add_fault(UvcPartialFrameFault('p'))
    transport.sent.clear()

    assert pipeline.clear() == 2
    assert frame_protocol.build_fault_clear(0x02) in transport.sent
    assert frame_protocol.build_fault_clear(0x03) in transport.sent


def test_without_transport_injection_fails_cleanly():
    pipeline = FaultPipeline()  # e.g. GPS bridge: no firmware peer
    with pytest.raises(ValueError, match='firmware transport'):
        pipeline.add_fault(UvcUsbDetachFault('f1'))
    assert not pipeline.has_faults


def test_process_is_passthrough():
    fault = UvcFrameDropFault('d')
    packets = [ScheduledPacket(0.0, b'\xff\xd8data')]
    assert fault.process(packets, 'video') == packets


def test_parameter_validation():
    with pytest.raises(FaultSpecError):
        UvcFrameDropFault('d', parameters={'percent': 0})
    with pytest.raises(FaultSpecError):
        UvcPartialFrameFault('p', parameters={'keep_percent': 100})
    with pytest.raises(FaultSpecError):
        UvcUsbDetachFault('u', parameters={'unknown_key': 1})


def test_i2c_fault_payloads():
    transport = FakeTransport()
    pipeline = FaultPipeline()
    pipeline.set_firmware_transport(transport)

    pipeline.add_fault(I2cResponseDelayFault(
        'd', parameters={'delay_us': 15000}))
    assert transport.sent[-1] == frame_protocol.build_fault_set(
        0x02, 15000, 0)

    pipeline.add_fault(I2cWhoAmIFault('w', parameters={'value': 0x71}))
    assert transport.sent[-1] == frame_protocol.build_fault_set(
        0x04, 0x71, 0)

    pipeline.add_fault(I2cNackFault('n'))
    assert transport.sent[-1] == frame_protocol.build_fault_set(0x01, 0, 0)

    assert pipeline.remove_fault('n')
    assert transport.sent[-1] == frame_protocol.build_fault_clear(0x01)


def test_i2c_parameter_validation():
    with pytest.raises(FaultSpecError):
        I2cResponseDelayFault('d', parameters={'delay_us': 0})
    with pytest.raises(FaultSpecError):
        I2cResponseDelayFault('d', parameters={'delay_us': 50001})
    with pytest.raises(FaultSpecError):
        I2cWhoAmIFault('w', parameters={'value': 256})
    with pytest.raises(FaultSpecError):
        I2cNackFault('n', parameters={'anything': 1})


def test_registered_for_scenario_validation():
    # The scenario runner validates specs via create_fault - firmware
    # fault types must resolve there without any node running.
    for fault_type in ('uvc_usb_detach', 'uvc_frame_drop',
                       'uvc_partial_frame', 'i2c_nack',
                       'i2c_response_delay', 'i2c_reg_freeze',
                       'i2c_who_am_i'):
        fault = create_fault(fault_type, 'x')
        assert fault.fault_type == fault_type
