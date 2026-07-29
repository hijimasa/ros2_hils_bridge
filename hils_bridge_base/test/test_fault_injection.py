"""Unit tests for the rclpy-independent fault injection core.

Run without ROS:
    cd hils_bridge_base && python3 -m pytest test/ -v
"""

import time

import pytest

from hils_bridge_base.fault_injection import (
    CorruptionFault, DelayedSender, DelayFault, DropFault, DuplicateFault,
    FaultPipeline, FaultSpecError, create_fault,
)

PKT = bytes(range(256)) * 4  # 1024-byte test packet


def run_sequence(pipeline, count=200, channel='data'):
    """Apply `count` packets and return the resulting plans."""
    return [pipeline.apply(PKT, channel) for _ in range(count)]


# -- passthrough / compatibility --

def test_empty_pipeline_is_passthrough():
    pipeline = FaultPipeline()
    plan = pipeline.apply(PKT)
    assert len(plan) == 1
    assert plan[0].delay_s == 0.0
    assert plan[0].data == PKT
    assert not pipeline.has_faults


def test_cleared_pipeline_returns_to_normal():
    pipeline = FaultPipeline()
    pipeline.add_fault(DropFault('d1', parameters={'probability': 1.0}))
    assert pipeline.apply(PKT) == []
    assert pipeline.clear() == 1
    plan = pipeline.apply(PKT)
    assert len(plan) == 1 and plan[0].data == PKT


# -- drop --

def test_drop_all():
    pipeline = FaultPipeline()
    pipeline.add_fault(DropFault('d1', parameters={'probability': 1.0}))
    assert all(plan == [] for plan in run_sequence(pipeline))
    stats = pipeline.snapshot()
    assert stats['dropped_packets'] == 200
    assert stats['output_packets'] == 0


def test_drop_probability_approximate():
    pipeline = FaultPipeline()
    pipeline.add_fault(DropFault('d1', seed=42,
                                 parameters={'probability': 0.3}))
    plans = run_sequence(pipeline, count=2000)
    dropped = sum(1 for p in plans if p == [])
    assert 480 <= dropped <= 720  # 0.3 +/- 0.06


def test_drop_every_n():
    pipeline = FaultPipeline()
    pipeline.add_fault(DropFault('d1', parameters={'every_n': 5}))
    plans = run_sequence(pipeline, count=20)
    dropped_idx = [i for i, p in enumerate(plans) if p == []]
    assert dropped_idx == [4, 9, 14, 19]


def test_drop_targets_channel():
    pipeline = FaultPipeline()
    pipeline.add_fault(
        DropFault('d1', target='data', parameters={'probability': 1.0}))
    assert pipeline.apply(PKT, 'data') == []
    plan = pipeline.apply(PKT, 'position')
    assert len(plan) == 1 and plan[0].data == PKT


# -- corrupt --

def test_corrupt_bit_flip_changes_data_keeps_length():
    pipeline = FaultPipeline()
    pipeline.add_fault(CorruptionFault(
        'c1', seed=7, parameters={'mode': 'bit_flip', 'num_bytes': 3}))
    for plan in run_sequence(pipeline, count=50):
        assert len(plan) == 1
        assert len(plan[0].data) == len(PKT)
        assert plan[0].data != PKT


def test_corrupt_zero_at_fixed_offset():
    fault = CorruptionFault(
        'c1', parameters={'mode': 'zero', 'num_bytes': 2, 'offset': 10})
    plan = fault.process([_scheduled(PKT)])
    assert plan[0].data[10:12] == b'\x00\x00'
    assert plan[0].data[:10] == PKT[:10]
    assert plan[0].data[12:] == PKT[12:]


def test_corrupt_truncate_shortens():
    pipeline = FaultPipeline()
    pipeline.add_fault(CorruptionFault(
        'c1', seed=3, parameters={'mode': 'truncate'}))
    for plan in run_sequence(pipeline, count=50):
        assert 1 <= len(plan[0].data) < len(PKT)


# -- delay --

def test_delay_fixed():
    pipeline = FaultPipeline()
    pipeline.add_fault(DelayFault('t1', parameters={'delay_ms': 250}))
    for plan in run_sequence(pipeline, count=10):
        assert plan[0].delay_s == pytest.approx(0.25)
        assert plan[0].data == PKT


def test_delay_uniform_jitter_bounds():
    pipeline = FaultPipeline()
    pipeline.add_fault(DelayFault(
        't1', seed=11,
        parameters={'delay_ms': 100, 'jitter_ms': 50,
                    'distribution': 'uniform'}))
    delays = [plan[0].delay_s for plan in run_sequence(pipeline, count=500)]
    assert all(0.05 <= d <= 0.15 for d in delays)
    assert max(delays) - min(delays) > 0.01  # actually jittering


# -- duplicate --

def test_duplicate_copies():
    pipeline = FaultPipeline()
    pipeline.add_fault(DuplicateFault('u1', parameters={'copies': 2}))
    plan = pipeline.apply(PKT)
    assert len(plan) == 3
    assert all(p.data == PKT for p in plan)


# -- reproducibility (docs section 17.1) --

def _fault_set(seed):
    return [
        DropFault('drop', seed=seed, parameters={'probability': 0.2}),
        CorruptionFault('corrupt', seed=seed,
                        parameters={'probability': 0.3, 'num_bytes': 2}),
        DelayFault('delay', seed=seed,
                   parameters={'delay_ms': 50, 'jitter_ms': 20}),
    ]


def _plan_signature(plans):
    return [tuple((p.delay_s, p.data) for p in plan) for plan in plans]


def test_same_seed_same_fault_sequence():
    p1, p2 = FaultPipeline(), FaultPipeline()
    for f in _fault_set(1001):
        p1.add_fault(f)
    for f in _fault_set(1001):
        p2.add_fault(f)
    assert _plan_signature(run_sequence(p1)) == \
        _plan_signature(run_sequence(p2))


def test_different_seed_different_sequence():
    p1, p2 = FaultPipeline(), FaultPipeline()
    for f in _fault_set(1001):
        p1.add_fault(f)
    for f in _fault_set(1002):
        p2.add_fault(f)
    assert _plan_signature(run_sequence(p1)) != \
        _plan_signature(run_sequence(p2))


# -- spec validation (docs section 18) --

def test_create_fault_unknown_type_rejected():
    with pytest.raises(FaultSpecError):
        create_fault('explode', 'x1')


def test_out_of_range_parameters_rejected():
    with pytest.raises(FaultSpecError):
        create_fault('drop', 'd1', parameters={'probability': 1.5})
    with pytest.raises(FaultSpecError):
        create_fault('delay', 't1', parameters={'delay_ms': -1})
    with pytest.raises(FaultSpecError):
        create_fault('corrupt', 'c1', parameters={'mode': 'nonsense'})


def test_unknown_parameter_rejected():
    with pytest.raises(FaultSpecError):
        create_fault('drop', 'd1', parameters={'probabillity': 0.5})


def test_duplicate_fault_id_rejected():
    pipeline = FaultPipeline()
    pipeline.add_fault(DropFault('d1'))
    with pytest.raises(ValueError):
        pipeline.add_fault(DropFault('d1'))


# -- delayed sender --

def test_delayed_sender_executes_in_due_order():
    sender = DelayedSender()
    results = []
    sender.submit(0.10, lambda: results.append('late'))
    sender.submit(0.02, lambda: results.append('early'))
    sender.submit(0.06, lambda: results.append('middle'))
    time.sleep(0.3)
    assert results == ['early', 'middle', 'late']
    assert sender.pending_count == 0
    sender.stop()


def test_delayed_sender_coarse_timing():
    sender = DelayedSender()
    done = []
    start = time.monotonic()
    sender.submit(0.1, lambda: done.append(time.monotonic() - start))
    time.sleep(0.3)
    assert len(done) == 1
    assert 0.08 <= done[0] <= 0.2  # loose bound: CI schedulers are noisy
    sender.stop()


def test_delayed_sender_stop_discards_pending():
    sender = DelayedSender()
    results = []
    sender.submit(5.0, lambda: results.append('should not run'))
    assert sender.pending_count == 1
    discarded = sender.stop()
    assert discarded == 1
    time.sleep(0.05)
    assert results == []
    assert sender.submit(0.0, lambda: None) is False  # stopped stays stopped


def test_delayed_sender_error_does_not_kill_thread():
    errors = []
    sender = DelayedSender(on_error=errors.append)
    results = []
    sender.submit(0.01, _raise_oserror)
    sender.submit(0.05, lambda: results.append('ok'))
    time.sleep(0.2)
    assert len(errors) == 1
    assert results == ['ok']
    sender.stop()


def _raise_oserror():
    raise OSError('boom')


def _scheduled(data):
    from hils_bridge_base.fault_injection import ScheduledPacket
    return ScheduledPacket(0.0, data)
