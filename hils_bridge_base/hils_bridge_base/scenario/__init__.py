"""
hils_bridge_base.scenario - YAML fault scenario execution.

rclpy-independent core:
  scenario_loader  - YAML loading and validation (rejects bad scenarios
                     before the test starts)
  event_scheduler  - timeline tracking with planned/actual firing times

ROS 2 integration:
  scenario_runner  - node driving a target emulator's fault services
"""

from .scenario_loader import (
    ACTIONS, Scenario, ScenarioError, ScenarioEvent,
    load_scenario_dict, load_scenario_file, load_scenario_yaml,
)
from .event_scheduler import EventSchedule, TrackedEvent

__all__ = [
    'ACTIONS', 'Scenario', 'ScenarioError', 'ScenarioEvent',
    'load_scenario_dict', 'load_scenario_file', 'load_scenario_yaml',
    'EventSchedule', 'TrackedEvent',
]
