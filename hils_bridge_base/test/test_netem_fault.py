"""Unit tests for the netem (tc qdisc) fault backend (docs 4.5).

tc itself is replaced with a recorder; the real command needs
CAP_NET_ADMIN and is exercised in the docker environment.
"""

import pytest

from hils_bridge_base.fault_injection import (
    FaultPipeline, FaultSpecError, NetemFault, create_fault,
)


@pytest.fixture(autouse=True)
def fake_tc(monkeypatch):
    calls = []
    monkeypatch.setattr(NetemFault, 'tc_runner',
                        staticmethod(lambda args: calls.append(args)))
    monkeypatch.setattr(NetemFault, '_active', {})
    return calls


def test_apply_and_delete_commands(fake_tc):
    pipeline = FaultPipeline()
    fault = NetemFault('n1', parameters={
        'interface': 'eth0', 'delay_ms': 100, 'jitter_ms': 20,
        'loss_percent': 5})
    pipeline.add_fault(fault)
    assert fake_tc == [['qdisc', 'replace', 'dev', 'eth0', 'root',
                        'netem', 'delay', '100.0ms', '20.0ms',
                        'loss', '5.0%']]
    assert pipeline.remove_fault('n1')
    assert fake_tc[-1] == ['qdisc', 'del', 'dev', 'eth0', 'root']


def test_tc_command_recorded_in_parameters():
    fault = NetemFault('n1', parameters={'delay_ms': 50})
    params = fault.parameters()
    assert params['tc_command'] == \
        'tc qdisc replace dev eth0 root netem delay 50.0ms'


def test_single_instance_per_interface(fake_tc):
    pipeline = FaultPipeline()
    pipeline.add_fault(NetemFault('n1', parameters={'delay_ms': 10}))
    with pytest.raises(ValueError, match='already active'):
        pipeline.add_fault(NetemFault('n2', parameters={'loss_percent': 1}))
    # A different interface is fine
    pipeline.add_fault(NetemFault(
        'n3', parameters={'interface': 'lo', 'delay_ms': 10}))


def test_failed_apply_releases_interface(monkeypatch):
    def boom(_args):
        raise ValueError('tc failed')
    monkeypatch.setattr(NetemFault, 'tc_runner', staticmethod(boom))
    monkeypatch.setattr(NetemFault, '_active', {})
    pipeline = FaultPipeline()
    with pytest.raises(ValueError, match='tc failed'):
        pipeline.add_fault(NetemFault('n1', parameters={'delay_ms': 10}))
    assert NetemFault._active == {}
    assert not pipeline.has_faults


def test_clear_survives_delete_failure(fake_tc, monkeypatch):
    pipeline = FaultPipeline()
    pipeline.add_fault(NetemFault('n1', parameters={'delay_ms': 10}))

    def boom(_args):
        raise ValueError('qdisc already gone')
    monkeypatch.setattr(NetemFault, 'tc_runner', staticmethod(boom))
    assert pipeline.clear() == 1
    assert NetemFault._active == {}


def test_validation():
    with pytest.raises(FaultSpecError, match='at least one effect'):
        NetemFault('n', parameters={'interface': 'eth0'})
    with pytest.raises(FaultSpecError, match='jitter'):
        NetemFault('n', parameters={'jitter_ms': 10})
    with pytest.raises(FaultSpecError, match='reorder'):
        NetemFault('n', parameters={'reorder_percent': 10})
    with pytest.raises(FaultSpecError, match='interface'):
        NetemFault('n', parameters={'interface': 'eth0; rm -rf /',
                                    'delay_ms': 1})
    with pytest.raises(FaultSpecError):
        NetemFault('n', parameters={'loss_percent': 101})
    with pytest.raises(FaultSpecError):
        NetemFault('n', parameters={'unknown': 1})


def test_registered_for_scenario_validation():
    fault = create_fault('netem', 'x', parameters={'delay_ms': 5})
    assert fault.fault_type == 'netem'


def test_process_is_passthrough():
    from hils_bridge_base.fault_injection.fault_base import ScheduledPacket
    fault = NetemFault('n', parameters={'delay_ms': 5})
    packets = [ScheduledPacket(0.0, b'data')]
    assert fault.process(packets, 'data') == packets
