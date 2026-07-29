"""Unit tests for the rclpy-independent fault injection core.

Run without ROS:
    cd hils_bridge_base && python3 -m pytest test/ -v
"""

import time

import pytest

from hils_bridge_base.fault_injection import (
    CorruptionFault, DelayedSender, DelayFault, DropFault, DuplicateFault,
    FaultPipeline, FaultSpecError, FreezeFault, HttpStatusFault,
    NmeaChecksumFault, ReorderFault, Wt901ChecksumFault, create_fault,
    register_fault_class,
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
    plan = fault.process([_scheduled(PKT)], 'data')
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


# -- freeze --

def test_freeze_repeats_first_packet_per_channel():
    pipeline = FaultPipeline()
    pipeline.add_fault(FreezeFault('f1'))
    first_a = pipeline.apply(b'aaa-1', 'a')[0].data
    first_b = pipeline.apply(b'bbb-1', 'b')[0].data
    assert first_a == b'aaa-1' and first_b == b'bbb-1'
    for i in range(5):
        assert pipeline.apply(f'aaa-{i + 2}'.encode(), 'a')[0].data == b'aaa-1'
        assert pipeline.apply(f'bbb-{i + 2}'.encode(), 'b')[0].data == b'bbb-1'


# -- reorder --

def test_reorder_reverse_groups():
    pipeline = FaultPipeline()
    pipeline.add_fault(ReorderFault(
        'r1', parameters={'group_size': 3, 'mode': 'reverse'}))
    assert pipeline.apply(b'p1') == []
    assert pipeline.apply(b'p2') == []
    plan = pipeline.apply(b'p3')
    assert [p.data for p in plan] == [b'p3', b'p2', b'p1']


def test_reorder_shuffle_is_seeded():
    def released(seed):
        pipeline = FaultPipeline()
        pipeline.add_fault(ReorderFault(
            'r1', seed=seed, parameters={'group_size': 8}))
        out = []
        for i in range(16):
            out.extend(p.data for p in pipeline.apply(bytes([i])))
        return out

    a, b = released(5), released(5)
    assert a == b
    assert sorted(a) == [bytes([i]) for i in range(16)]  # nothing lost
    assert released(5) != released(6)


# -- http_status --

def test_http_status_overrides_status_not_body():
    pipeline = FaultPipeline()
    fault = HttpStatusFault('h1', target='http',
                            parameters={'status': 503})
    pipeline.add_fault(fault)
    # Body passes through the pipeline unchanged.
    plan = pipeline.apply(b'{"ok": true}', 'http')
    assert plan[0].data == b'{"ok": true}'
    # Status sampled out of band.
    assert fault.sample_status() == 503
    assert fault.applied_count == 1


def test_http_status_probability_seeded():
    def samples(seed):
        fault = HttpStatusFault('h1', seed=seed,
                                parameters={'status': 500,
                                            'probability': 0.5})
        return [fault.sample_status() for _ in range(50)]

    assert samples(9) == samples(9)
    hits = [s for s in samples(9) if s == 500]
    assert 10 <= len(hits) <= 40


def test_faults_for_filters_by_channel():
    pipeline = FaultPipeline()
    pipeline.add_fault(DropFault('d1', target='data'))
    pipeline.add_fault(HttpStatusFault('h1', target='http'))
    pipeline.add_fault(DelayFault('t1'))  # all channels
    ids = [f.fault_id for f in pipeline.faults_for('http')]
    assert ids == ['h1', 't1']


# -- protocol faults --

NMEA = (b'$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,'
        b'46.9,M,,*47\r\n'
        b'$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,'
        b'003.1,W*6A\r\n')


def test_nmea_checksum_invalidated_only():
    pipeline = FaultPipeline()
    pipeline.add_fault(NmeaChecksumFault('n1'))
    out = pipeline.apply(NMEA, 'serial')[0].data
    assert out != NMEA
    assert len(out) == len(NMEA)
    assert b'*48' in out and b'*6B' in out  # each checksum +1
    # Everything except the checksum hex digits is untouched.
    assert out.replace(b'*48', b'*47').replace(b'*6B', b'*6A') == NMEA


def test_nmea_checksum_zero_probability_passthrough():
    fault = NmeaChecksumFault('n1', parameters={'probability': 0.0})
    out = fault.process([_scheduled(NMEA)], 'serial')[0].data
    assert out == NMEA


def _wt901_frame(ptype):
    body = bytes([0x55, ptype] + list(range(8)))
    return body + bytes([sum(body) & 0xFF])


def test_wt901_checksum_corrupts_each_frame():
    payload = (_wt901_frame(0x51) + _wt901_frame(0x52)
               + _wt901_frame(0x53) + _wt901_frame(0x59))
    fault = Wt901ChecksumFault('w1')
    out = fault.process([_scheduled(payload)], 'serial')[0].data
    assert len(out) == len(payload)
    for i in range(4):
        frame_in = payload[i * 11:(i + 1) * 11]
        frame_out = out[i * 11:(i + 1) * 11]
        assert frame_out[:10] == frame_in[:10]        # data intact
        assert frame_out[10] == (frame_in[10] + 1) % 256  # checksum bad


def test_register_fault_class():
    class MyFault(DropFault):
        fault_type = 'my_device_fault'

    register_fault_class(MyFault)
    fault = create_fault('my_device_fault', 'x1',
                         parameters={'probability': 1.0})
    assert isinstance(fault, MyFault)
    with pytest.raises(TypeError):
        register_fault_class(dict)


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
