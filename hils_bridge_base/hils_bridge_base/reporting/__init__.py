"""
hils_bridge_base.reporting - Machine-readable test result output.

  report_writer - JSON and JUnit XML writers (rclpy-independent)
"""

from .report_writer import build_report, write_json, write_junit_xml

__all__ = ['build_report', 'write_json', 'write_junit_xml']
