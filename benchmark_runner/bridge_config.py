"""Set the plant's configured tire friction by rewriting the bridge config.

`vehicle.physics.wheels[*].tire_friction` in the bridge's
`carla_interface_config.yaml` is what `on_configure` loads, so a scenario gets
its own surface by patching the file before the runner configures the bridge.

The rewrite is a line-scoped regex rather than a YAML round-trip on purpose: the
config is a hand-maintained file whose comments carry the physics rationale, and
`yaml.safe_dump` would drop every one of them. This touches four lines and
leaves the file byte-identical everywhere else.
"""
import math
import re
from pathlib import Path

BRIDGE_CONFIG = Path('/home/ebrahim/Carla_ASU_Bridge/config/carla_interface_config.yaml')

N_WHEELS = 4

# Anchored at the line start so it matches only the four wheel entries, never
# `control_tire_friction:`, `tire_friction_configured`, or the commented
# references elsewhere in the file.
_WHEEL_MU = re.compile(r'^([ \t]+tire_friction:[ \t]*)[-+0-9.eE]+', re.MULTILINE)


def set_tire_friction(mu, path=BRIDGE_CONFIG):
    """Rewrite the four wheel coefficients in place."""
    mu = float(mu)
    if not math.isfinite(mu) or mu <= 0.0:
        raise ValueError(f'tire friction must be finite and > 0, got {mu!r}')
    # str(float) not :g - :g renders 1.0 as "1", turning the float into an int.
    patched, n = _WHEEL_MU.subn(rf'\g<1>{mu}', path.read_text(encoding='utf-8'))
    if n != N_WHEELS:
        raise RuntimeError(
            f'expected {N_WHEELS} wheel tire_friction lines in {path}, matched {n}')
    path.write_text(patched, encoding='utf-8')


def restore(text, path=BRIDGE_CONFIG):
    """Put back a snapshot taken before the first patch."""
    path.write_text(text, encoding='utf-8')
