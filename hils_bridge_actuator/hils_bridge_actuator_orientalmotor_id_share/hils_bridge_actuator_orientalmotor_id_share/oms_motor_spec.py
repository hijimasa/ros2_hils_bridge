"""Per-motor configuration and unit conversion for the ID-share bridge.

Python port of REACT-simulator's orientalmotor_id_share_emulator
oms_motor_spec.{hpp,cpp}. The OrientalMotor BLV-R and AZ series share
one ID-share Modbus protocol but differ in their driver-native units:

    BLV-R : velocity rpm, 36000 step/rev, torque as 0.1% of rated torque
    AZ    : velocity Hz,  1000  step/rev, torque as 0.1% of rated torque

parse_motor_types() accepts the SAME motor list syntax as
orientalmotor_id_share_daemon's --motor-types (e.g. "BLVR:100,AZ:60");
the list length sets the motor count. The KEY is the BLV-R output
power [W] or the AZ frame size [mm], used to look up the rated torque
for the Nm <-> 0.1%-of-rated conversion.

This module also holds the motor-shaft <-> output-shaft (joint)
helpers, applying the per-motor gear ratio, so the bridge node stays a
thin transport layer. No ROS imports.
"""

import enum
import math
from dataclasses import dataclass
from typing import List

# Per-series characteristics (values match REACT-simulator's
# orientalmotor_id_share_data.hpp).
BLVR_STEP_PER_REV = 36000
AZ_STEP_PER_REV = 1000
BLVR_MAX_RPM = 4000  # motor-shaft cap
AZ_MAX_HZ = 4000     # native cap (==> AZ_MAX_HZ*60/1000 rpm)

# Driver torque granularity: 1 LSB = 0.1% of rated, so 1000 LSB == 100%.
TORQUE_PERMILLE_FULL_SCALE = 1000

# BLV-R: output power [W] -> motor-shaft rated torque [Nm] at 3000 rpm:
# T = P / (2*pi*3000/60). Matches the published ratings (100 W = 0.32 N*m, ...).
_BLVR_WATT_TABLE = {
    60: 0.19,
    100: 0.32,
    200: 0.64,
    400: 1.27,
}

# AZ: frame size [mm] -> max holding torque [Nm] (standard motor, no gearhead).
_AZ_FRAME_TABLE = {
    20: 0.036,
    28: 0.19,
    42: 0.30,
    60: 1.0,
    85: 4.0,
}


class MotorType(enum.Enum):
    UNKNOWN = 0
    BLVR = 1  # BLV-R series: velocity rpm, 36000 step/rev
    AZ = 2    # AZ series:    velocity Hz,  1000 step/rev


@dataclass
class MotorSpec:
    """Resolved configuration for a single motor on the ID-share bus."""

    type: MotorType = MotorType.UNKNOWN
    key: int = 0                  # BLV-R: output power [W]; AZ: frame size [mm]
    rated_torque_nm: float = 0.0  # looked up from key; <= 0 means unknown
    step_per_rev: int = 0         # BLV-R 36000; AZ 1000


def motor_type_name(motor_type: MotorType) -> str:
    if motor_type == MotorType.BLVR:
        return 'BLV-R'
    if motor_type == MotorType.AZ:
        return 'AZ'
    return 'UNKNOWN'


def blvr_rated_torque_from_watt(watt: int) -> float:
    """Rated motor-shaft torque [Nm] of a BLV-R motor from output power [W]."""
    return _BLVR_WATT_TABLE.get(watt, 0.0)


def az_rated_torque_from_frame(frame_mm: int) -> float:
    """Max holding torque [Nm] of an AZ motor from frame size [mm]."""
    return _AZ_FRAME_TABLE.get(frame_mm, 0.0)


def step_per_rev(motor_type: MotorType) -> int:
    """Steps per revolution for a motor type (0 if unknown)."""
    if motor_type == MotorType.BLVR:
        return BLVR_STEP_PER_REV
    if motor_type == MotorType.AZ:
        return AZ_STEP_PER_REV
    return 0


def max_rpm_for_spec(spec: MotorSpec) -> float:
    """Per-type maximum motor-shaft speed [rpm], used to clamp commands."""
    if spec.type == MotorType.AZ and spec.step_per_rev > 0:
        return float(AZ_MAX_HZ) * 60.0 / spec.step_per_rev
    return float(BLVR_MAX_RPM)


def parse_motor_types(motor_types: str) -> List[MotorSpec]:
    """Parse a "motor_types" list such as "BLVR:100,AZ:60,BLVR:200".

    Each comma-separated entry is "TYPE[:KEY]": TYPE is "BLVR"/"BLV-R"
    or "AZ" (case-insensitive); KEY is BLV-R output [W] or AZ frame
    [mm]. If KEY is omitted the motor is still controlled but
    rated_torque_nm stays 0 (torque reported 0). Same syntax as
    orientalmotor_id_share_daemon's --motor-types.

    Raises:
        ValueError: on malformed input, with a human-readable message.
    """
    if not motor_types.strip():
        raise ValueError('motor-types list is empty')

    specs = []
    for index, token in enumerate(motor_types.split(',')):
        token = token.strip()
        if not token:
            raise ValueError(f'empty motor entry at position {index}')

        type_str, _, key_str = token.partition(':')
        type_str = type_str.strip()
        key_str = key_str.strip()

        spec = MotorSpec()
        type_upper = type_str.upper()
        if type_upper in ('BLVR', 'BLV-R'):
            spec.type = MotorType.BLVR
        elif type_upper == 'AZ':
            spec.type = MotorType.AZ
        else:
            raise ValueError(
                f'unknown motor type "{type_str}" at position {index}')

        spec.step_per_rev = step_per_rev(spec.type)

        if key_str:
            try:
                key = int(key_str)
                if key <= 0:
                    raise ValueError
            except ValueError:
                raise ValueError(
                    f'invalid key "{key_str}" at position {index}') from None
            spec.key = key

            if spec.type == MotorType.BLVR:
                spec.rated_torque_nm = blvr_rated_torque_from_watt(key)
                if spec.rated_torque_nm <= 0.0:
                    raise ValueError(
                        f'unknown BLV-R output power {key_str} W '
                        f'at position {index}')
            else:  # AZ
                spec.rated_torque_nm = az_rated_torque_from_frame(key)
                if spec.rated_torque_nm <= 0.0:
                    raise ValueError(
                        f'unknown AZ frame size {key_str} mm '
                        f'at position {index}')

        specs.append(spec)

    return specs


# ---- motor-shaft physical units <-> driver native units ---------------------

def rpm_to_native(rpm: float, spec: MotorSpec) -> float:
    """[rpm] -> native velocity (BLV-R rpm; AZ Hz = rpm/60*step_per_rev)."""
    if spec.type == MotorType.AZ:
        # Hz (pulses/sec) = rev/sec * step/rev = rpm/60 * step_per_rev
        return rpm / 60.0 * spec.step_per_rev
    return rpm  # BLV-R native unit is rpm


def native_to_rpm(native: float, spec: MotorSpec) -> float:
    """Native velocity -> [rpm]."""
    if spec.type == MotorType.AZ:
        if spec.step_per_rev <= 0:
            return 0.0
        return native * 60.0 / spec.step_per_rev
    return native


def permille_to_nm(permille: int, rated_torque_nm: float) -> float:
    """Driver torque (0.1% of rated) -> [Nm]."""
    if rated_torque_nm <= 0.0:
        return 0.0
    return permille / TORQUE_PERMILLE_FULL_SCALE * rated_torque_nm


def nm_to_permille(nm: float, rated_torque_nm: float) -> int:
    """[Nm] -> driver torque (0.1% of rated, rounded)."""
    if rated_torque_nm <= 0.0:
        return 0
    return round(nm / rated_torque_nm * TORQUE_PERMILLE_FULL_SCALE)


# ---- motor-shaft <-> output-shaft (joint) helpers, applying the gear ratio --

def motor_step_to_output_rad(steps: int, spec: MotorSpec, gear: float) -> float:
    """Motor position [step] -> output-shaft joint position [rad]."""
    if spec.step_per_rev <= 0 or gear == 0.0:
        return 0.0
    # motor_rad = steps * 2pi / step_per_rev; output_rad = motor_rad / gear.
    return steps * 2.0 * math.pi / spec.step_per_rev / gear


def output_rad_to_motor_step(out_rad: float, spec: MotorSpec, gear: float) -> int:
    """Output-shaft joint position [rad] -> motor position [step, rounded]."""
    if spec.step_per_rev <= 0:
        return 0
    # motor_steps = output_rad * gear * step_per_rev / 2pi.
    return round(out_rad * gear * spec.step_per_rev / (2.0 * math.pi))


def native_vel_to_output_rad_s(native: int, spec: MotorSpec, gear: float) -> float:
    """Native motor velocity -> output-shaft joint velocity [rad/s]."""
    if gear == 0.0:
        return 0.0
    motor_rpm = native_to_rpm(float(native), spec)
    return motor_rpm * 2.0 * math.pi / 60.0 / gear


def output_rad_s_to_native_vel(out_rad_s: float, spec: MotorSpec, gear: float) -> int:
    """Output-shaft joint velocity [rad/s] -> native motor velocity (rounded)."""
    motor_rpm = out_rad_s * gear * 60.0 / (2.0 * math.pi)
    return round(rpm_to_native(motor_rpm, spec))
