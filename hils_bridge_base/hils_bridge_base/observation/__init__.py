"""
hils_bridge_base.observation - Test oracle observation and evaluation.

rclpy-independent core:
  expectations - pass/fail evaluators for scenario expectations

ROS 2 integration:
  recorder     - read-only topic arrival recorder (robot domain)
  oracle_node  - scenario_oracle executable (dual-domain oracle)
"""

from .expectations import (
    ERROR, FAIL, PASS, SKIP, SUPPORTED_TYPES, Verdict, evaluate,
)

__all__ = [
    'ERROR', 'FAIL', 'PASS', 'SKIP', 'SUPPORTED_TYPES', 'Verdict',
    'evaluate',
]
