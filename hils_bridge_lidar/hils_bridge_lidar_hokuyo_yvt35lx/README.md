# hils_bridge_lidar_hokuyo_yvt35lx

Pure-software emulator for the Hokuyo **YVT-35LX** 3D LiDAR (3D-URG).
Speaks the **VSSP 2.1** protocol over TCP (port 10940) so the real
[urg3d_node2](https://github.com/Hokuyo-aut/urg3d_node2) driver can be
evaluated without hardware. Point clouds from any `PointCloud2` source
(synthetic generator, rosbag, or a live simulator) are re-scanned into
the YVT line/spot pattern (36 lines x 74 spots) and streamed as
`_ri`/`_ro` range packets; `_ax` auxiliary packets carry IMU /
compass / temperature records.

The packet layout was written against `urg3d_library` (the driver's
protocol parser), covering:

| Area | Commands / packets |
|------|--------------------|
| Identification | `VER` (vendor / product / serial / firmware / `prot:VSSP2.1`) |
| Status | `GET:stat` (`_ro`/`_ri`/`_ax` = `000`) |
| Angle tables | `GET:tblh`, `GET:tv00`..`GET:tv09` |
| Interlace | `GET:_itl`, `GET:_itv`, `SET:_itl=0,NN`, `SET:_itv=0,NN` |
| Measurement | `DAT:ro/ri/ax=0/1`, `_ri`/`_ro` line packets, `_ax` packets |
| Misc | `RST` |

Since the protocol version reports `VSSP2.1`, urg3d_library uses its
built-in YVT-35LX constants (74 spots, lines 0..35, max echo 4) — the
same grid this emulator generates.

## Usage

Loopback (no extra NIC, no simulator — see
`tools/run_yvt35lx_e2e.sh` for the full E2E):

```bash
ros2 run hils_bridge_lidar_hokuyo_yvt35lx yvt35lx_emulator_node \
    --ros-args -p device_ip:=127.0.0.1 -p host_ip:=127.0.0.1 \
    -p pointcloud_topic:=/sim_points

ros2 run urg3d_node2 urg3d_node2_node --ros-args -p ip_address:=127.0.0.1
ros2 lifecycle set /urg3d_node2 configure
ros2 lifecycle set /urg3d_node2 activate
```

USB-Ethernet adapter (real network path, sensor default IP):

```bash
sudo ip addr add 192.168.0.10/24 dev eth1
ros2 launch hils_bridge_lidar_hokuyo_yvt35lx yvt35lx_emulator.launch.py \
    network_interface:=eth1 host_ip:=192.168.0.15
```

## Sensor noise

A real device's error lands on the measured distance along the beam, so
it is applied to the millimetre range values that go into the packet -
not to the cartesian points coming in.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `range_noise_sigma_m` | `0.0` | Std-dev of the range noise, in metres |
| `dropout_probability` | `0.0` | Chance that a spot reports no echo |
| `noise_seed` | `0` | Same seed + same call sequence = same output |

**Off by default on purpose**: the simulation feeding the emulator
normally models its own sensor noise, and applying it on both sides
would misrepresent what the driver really sees. Turn it on only when
the point source is noise-free — a synthetic scene, or a clean rosbag.
`tools/run_yvt35lx_demo.sh` does exactly that (20 mm, 1 % dropout).

Noise is drawn once per revolution, so a static scene still yields an
independent measurement per scan. A return pushed outside the 0.3-35 m
range, or dropped, is reported as no echo at all, and its intensity
goes with it.

```bash
ros2 run hils_bridge_lidar_hokuyo_yvt35lx yvt35lx_emulator_node --ros-args \
    -p range_noise_sigma_m:=0.02 -p dropout_probability:=0.01 -p noise_seed:=1
```

Pass `0.0`, not `0`: ROS types a bare `0` as an integer and refuses to
assign it to these double parameters.

## Notes / limitations

- **Interlace**: `SET:_itl`/`SET:_itv` are accepted and motor/rem
  field numbers plus frame counters cycle accordingly, but all fields
  sample the same angle grid (no sub-bin angular offsets). Driver
  interlace bookkeeping is exercised; angular super-resolution is not.
- **Corruption faults on TCP crash urg3d_node2** (found 2026-08-04,
  see policy doc 22.5): byte corruption on the `data` channel
  segfaults the driver inside
  `Urg3dSensor::setCommonMeasurementData` within seconds, because
  `urg3d_library` validates range-header fields only after indexing
  with them. `yvt35lx_stream_fault_001` therefore FAILs today and is
  kept as the regression for that defect; `yvt35lx_blackout_001` is
  the green path. `tools/probe_urg3d_range_header.py` injects one
  crafted field at a time to narrow such crashes down.
- **Multiple connections** are accepted; measurement streams are
  per-connection, interlace settings are device-global (as on the real
  sensor).
- **Points the real sensor could not see are dropped**: outside the
  0.3–35 m datasheet range or outside the configured FOV, a spot gets
  no echo rather than a fabricated one.
- Auxiliary (`_ax`) values are constant (1 g on Z, fixed compass and
  temperature) scaled per `urg3d_node2/auxiliary_define.hpp`.

## Tests

`python3 -m pytest test -q` (12 tests) re-parses the encoder's output
the way `urg3d_library` does, so a layout change that would break the
real driver fails here before the E2E does.
