"""Patching the bridge config must change four numbers and nothing else.

The real config is copied into tmp_path, so these run without touching the
bridge repository.
"""
import shutil
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from bridge_config import BRIDGE_CONFIG  # noqa: E402
from bridge_config import N_WHEELS  # noqa: E402
from bridge_config import restore  # noqa: E402
from bridge_config import set_tire_friction  # noqa: E402


@pytest.fixture
def config(tmp_path):
    if not BRIDGE_CONFIG.exists():
        pytest.skip(f'{BRIDGE_CONFIG} not present')
    path = tmp_path / BRIDGE_CONFIG.name
    shutil.copy(BRIDGE_CONFIG, path)
    return path


def _wheel_values(text):
    return [line.split(':', 1)[1].strip()
            for line in text.splitlines()
            if line[:1] in ' \t' and line.strip().startswith('tire_friction:')]


@pytest.mark.parametrize('mu, written', [(1.2, '1.2'), (1.0, '1.0'), (0.75, '0.75')])
def test_patches_every_wheel_as_a_float(config, mu, written):
    set_tire_friction(mu, config)
    assert _wheel_values(config.read_text(encoding='utf-8')) == [written] * N_WHEELS


def test_leaves_everything_but_the_wheels_byte_identical(config):
    original = config.read_text(encoding='utf-8')
    set_tire_friction(1.2, config)
    patched = config.read_text(encoding='utf-8')


    before = original.splitlines()
    after = patched.splitlines()
    assert len(before) == len(after)
    differing = [(a, b) for a, b in zip(before, after) if a != b]
    assert len(differing) == N_WHEELS
    assert all('tire_friction' in a for a, _ in differing)


def test_restore_round_trips(config):
    original = config.read_text(encoding='utf-8')
    set_tire_friction(0.8, config)
    restore(original, config)
    assert config.read_text(encoding='utf-8') == original


def test_missing_wheels_block_raises(config):
    config.write_text('vehicle:\n  physics:\n    wheels: []\n', encoding='utf-8')
    with pytest.raises(RuntimeError):
        set_tire_friction(1.2, config)


@pytest.mark.parametrize('mu', [0.0, -1.0, float('nan'), float('inf')])
def test_rejects_unusable_friction(config, mu):
    original = config.read_text(encoding='utf-8')
    with pytest.raises(ValueError):
        set_tire_friction(mu, config)
    assert config.read_text(encoding='utf-8') == original
