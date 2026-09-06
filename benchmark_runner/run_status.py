"""The sidecar a failed scenario leaves behind, and how to read it back.

`run_benchmark.py` writes one of these into each benchmark node's
`<plot_output_dir>/raw/` when it gives up on a scenario. The nodes read it while
exporting, so a failed run's figures can mark where the data stops, and
`compare_scenarios.py` reads it so a short run is not silently compared against
complete ones.

No ROS imports here on purpose: compare_scenarios.py runs standalone.

The two benchmark node packages cannot import this module (they are separate
ROS packages), so they carry their own copy of the filename and the reader -
keep `STATUS_FILENAME` and the schema in step with them.
"""
import json
from pathlib import Path

STATUS_FILENAME = 'run_status.json'


def write_failure(directory, reason):
    """Record why the runner gave up. Raises OSError if the directory is gone."""
    path = Path(directory) / 'raw' / STATUS_FILENAME
    path.write_text(
        json.dumps({'status': 'failed', 'reason': reason}), encoding='utf-8')
    return path


def read_failure_reason(directory):
    """Why the runner gave up on this scenario, or None if it didn't.

    The runner wipes the directory at scenario start, so a stale file from an
    earlier run cannot appear: the file's presence means this run failed.
    """
    try:
        status = json.loads(
            (Path(directory) / 'raw' / STATUS_FILENAME).read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return None
    return status.get('reason') if status.get('status') == 'failed' else None
