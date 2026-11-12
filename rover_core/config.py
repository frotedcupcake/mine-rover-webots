"""Load config/rover.yaml."""
import copy
import os

import yaml

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
DEFAULT_PATH = os.path.join(ROOT, "config", "rover.yaml")


def load(path=None, overrides=None):
    """Return the config as a nested dict. `overrides` is merged on top (nested)."""
    with open(path or DEFAULT_PATH) as fh:
        cfg = yaml.safe_load(fh)
    if overrides:
        cfg = merge(cfg, overrides)
    return cfg


def merge(base, extra):
    out = copy.deepcopy(base)
    for k, v in (extra or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = merge(out[k], v)
        else:
            out[k] = v
    return out
