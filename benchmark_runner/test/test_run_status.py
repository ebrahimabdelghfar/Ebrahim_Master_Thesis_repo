"""The failure sidecar the runner writes and the benchmark nodes read.

Both sides are unbound calls: neither `_record_failure` nor `_read_failure_reason`
touches `self`, so this needs no ROS graph. The point of the test is that the two
halves agree on the filename and the schema - they live in three packages that
cannot import each other.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import compare_scenarios  # noqa: E402
from run_benchmark import BenchmarkRunner  # noqa: E402
from run_status import STATUS_FILENAME  # noqa: E402
from run_status import read_failure_reason  # noqa: E402
from run_status import write_failure  # noqa: E402

REASON = 'only 1/3 timed laps in 1200s'


def _readers():
    """The nodes' readers, skipped individually if their overlay is not sourced."""
    found = []
    for module, cls in (
        ('adaptive_controller_benchmark.adaptive_controller_benchmark_node',
         'AdaptiveControllerBenchmarkNode'),
        ('tire_force_benchmark.tire_force_benchmark_node',
         'TireForceBenchmarkNode'),
    ):
        try:
            found.append(getattr(__import__(module, fromlist=[cls]), cls))
        except ImportError:
            pass
    return found


READERS = _readers()
requires_nodes = pytest.mark.skipif(
    not READERS, reason='benchmark node packages are not on the path')


@pytest.fixture
def dirs(tmp_path):
    made = [tmp_path / 'identification' / 'ours', tmp_path / 'control' / 'ours']
    for directory in made:
        (directory / 'raw').mkdir(parents=True)
    return made


def test_records_the_reason_in_every_output_dir(dirs):
    BenchmarkRunner._record_failure(None, dirs, REASON)
    for directory in dirs:
        assert (directory / 'raw' / STATUS_FILENAME).exists()
        # compare_scenarios.py reads it through this same helper.
        assert read_failure_reason(directory) == REASON


@requires_nodes
@pytest.mark.parametrize('cls', READERS, ids=lambda c: c.__name__)
def test_nodes_read_back_what_the_runner_wrote(dirs, cls):
    BenchmarkRunner._record_failure(None, dirs, REASON)
    for directory in dirs:
        assert cls._read_failure_reason(None, directory) == REASON


@requires_nodes
@pytest.mark.parametrize('cls', READERS, ids=lambda c: c.__name__)
def test_clean_run_has_no_reason(dirs, cls):
    # Nothing written: a scenario that succeeded leaves no status file.
    for directory in dirs:
        assert cls._read_failure_reason(None, directory) is None


@requires_nodes
@pytest.mark.parametrize('cls', READERS, ids=lambda c: c.__name__)
def test_unreadable_status_is_not_a_failure(dirs, cls):
    directory = dirs[0]
    (directory / 'raw' / STATUS_FILENAME).write_text('{truncated', encoding='utf-8')
    assert cls._read_failure_reason(None, directory) is None


@requires_nodes
@pytest.mark.parametrize('cls', READERS, ids=lambda c: c.__name__)
def test_non_failed_status_is_not_a_failure(dirs, cls):
    directory = dirs[0]
    (directory / 'raw' / STATUS_FILENAME).write_text(
        '{"status": "ok", "reason": "should be ignored"}', encoding='utf-8')
    assert cls._read_failure_reason(None, directory) is None


# ---------------- the cross-scenario comparison ----------------

def test_comparison_picks_the_reason_up_from_either_dir(tmp_path):
    root = tmp_path / 'graphs'
    for kind in ('identification', 'control'):
        (root / kind / 'ours' / 'raw').mkdir(parents=True)
    write_failure(root / 'identification' / 'ours', REASON)
    assert compare_scenarios.ScenarioData('ours', root).failure_reason == REASON


def test_comparison_reports_a_clean_run_as_complete(tmp_path):
    root = tmp_path / 'graphs'
    for kind in ('identification', 'control'):
        (root / kind / 'ours' / 'raw').mkdir(parents=True)
    assert compare_scenarios.ScenarioData('ours', root).failure_reason is None


@pytest.mark.parametrize('incomplete, expected_texts', [((), 0), ((('ours', REASON),), 1)])
def test_every_comparison_figure_carries_the_caveat(tmp_path, incomplete, expected_texts):
    matplotlib = pytest.importorskip('matplotlib')
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    cmp = compare_scenarios.Comparison(tmp_path, plt, incomplete=incomplete)
    fig, ax = plt.subplots()
    ax.plot([0, 1], [0, 1])
    cmp.save(fig, 'figure', ['a'], [[1]])

    assert len(fig.texts) == expected_texts
    if expected_texts:
        assert REASON in fig.texts[0].get_text()
    for suffix in ('png', 'pdf', 'csv'):
        assert (tmp_path / f'figure.{suffix}').is_file()
