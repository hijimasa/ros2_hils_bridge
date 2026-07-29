"""Device state definitions (docs section 4.2).

Standard states model what is observable on the public interface of a
real device. Not every emulated device uses every state; the default
channel policy below defines which logical send channels are open in
each state, and devices can override it.
"""

POWER_OFF = 'power_off'
BOOTING = 'booting'
DISCOVERABLE = 'discoverable'
CONFIGURING = 'configuring'
READY = 'ready'
STREAMING = 'streaming'

DEGRADED = 'degraded'
COMMUNICATION_FAULT = 'communication_fault'
CONFIGURATION_FAULT = 'configuration_fault'
INTERNAL_ERROR = 'internal_error'
REBOOTING = 'rebooting'

ALL_STATES = (
    POWER_OFF, BOOTING, DISCOVERABLE, CONFIGURING, READY, STREAMING,
    DEGRADED, COMMUNICATION_FAULT, CONFIGURATION_FAULT, INTERNAL_ERROR,
    REBOOTING,
)

# Pseudo-state accepted by set_device_state: enters REBOOTING and
# auto-transitions to the configured reboot target after boot_duration.
REBOOT_REQUEST = 'reboot'

# Channels that carry measurement data. Any channel not listed in a
# state's open set is treated as a data channel and blocked outside
# STREAMING / DEGRADED.
_ALL = 'all'

DEFAULT_CHANNEL_POLICY = {
    POWER_OFF: frozenset(),
    BOOTING: frozenset(),
    REBOOTING: frozenset(),
    COMMUNICATION_FAULT: frozenset(),
    INTERNAL_ERROR: frozenset(),
    # Discovery must also answer configuration commands so real drivers
    # (e.g. livox_ros_driver2) can bring the device back up.
    DISCOVERABLE: frozenset({'discovery', 'command'}),
    CONFIGURING: frozenset({'discovery', 'command'}),
    CONFIGURATION_FAULT: frozenset({'discovery', 'command'}),
    READY: frozenset({'discovery', 'command', 'status', 'position'}),
    STREAMING: _ALL,
    DEGRADED: _ALL,
}


def is_valid_state(state: str) -> bool:
    return state in ALL_STATES


def channel_open(policy: dict, state: str, channel: str) -> bool:
    allowed = policy.get(state, frozenset())
    if allowed == _ALL:
        return True
    return channel in allowed
