"""Machine-readable test reports (docs section 6.5).

Independent of rclpy. Writes the oracle's result as:
  - JSON: full detail for archival and downstream tooling
  - JUnit XML: CI integration (one testcase per expectation)
"""

import json
import os
from typing import List
from xml.sax.saxutils import escape, quoteattr

from hils_bridge_base.observation.expectations import (
    ERROR, FAIL, PASS, SKIP, Verdict,
)


def build_report(*, scenario: dict, verdicts: List[Verdict],
                 observations: dict, environment: dict) -> dict:
    """Assemble the result document (docs section 4.4 metadata list)."""
    counts = {status: 0 for status in (PASS, FAIL, SKIP, ERROR)}
    for v in verdicts:
        counts[v.status] = counts.get(v.status, 0) + 1
    return {
        'scenario': scenario,
        'environment': environment,
        'observations': observations,
        'expectations': [
            {
                'index': v.index,
                'type': v.exp_type,
                'status': v.status,
                'detail': v.detail,
                'expectation': v.expectation,
            } for v in verdicts
        ],
        'summary': {
            **counts,
            'total': len(verdicts),
            'passed': all(v.passed for v in verdicts),
        },
    }


def write_json(report: dict, path: str) -> None:
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False, default=str)
        f.write('\n')


def write_junit_xml(report: dict, path: str) -> None:
    scenario_id = report['scenario'].get('id', 'scenario')
    cases = []
    failures = 0
    errors = 0
    skipped = 0
    for exp in report['expectations']:
        name = f"[{exp['index']}] {exp['type']}"
        body = ''
        if exp['status'] == FAIL:
            failures += 1
            body = (f"    <failure message={quoteattr(exp['detail'])}>"
                    f"{escape(json.dumps(exp['expectation']))}</failure>\n")
        elif exp['status'] == ERROR:
            errors += 1
            body = (f"    <error message={quoteattr(exp['detail'])}>"
                    f"{escape(json.dumps(exp['expectation']))}</error>\n")
        elif exp['status'] == SKIP:
            skipped += 1
            body = f"    <skipped message={quoteattr(exp['detail'])}/>\n"
        cases.append(
            f"  <testcase classname={quoteattr(scenario_id)} "
            f"name={quoteattr(name)}>\n{body}  </testcase>\n")

    xml = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f'<testsuite name={quoteattr(scenario_id)} '
        f'tests="{len(cases)}" failures="{failures}" '
        f'errors="{errors}" skipped="{skipped}">\n'
        + ''.join(cases) + '</testsuite>\n')

    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        f.write(xml)
