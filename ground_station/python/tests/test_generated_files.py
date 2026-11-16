"""Generated artefacts must be up to date with their sources."""
from __future__ import annotations

import importlib.util
from pathlib import Path

GS_ROOT = Path(__file__).resolve().parents[2]


def _load(path: Path):
    spec = importlib.util.spec_from_file_location(path.stem, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_js_cross_check_vectors_are_fresh():
    """web/test/test_vectors.json == what rover_core produces now (rerun tools/gen_vectors.py)."""
    gen = _load(GS_ROOT / "tools" / "gen_vectors.py")
    assert gen.DEFAULT_OUT.read_text(encoding="utf-8") == gen.render()


def test_web_bundle_is_fresh():
    """web/js/gs-bundle.js (file:// fallback) == bundle of the ES modules (rerun tools/bundle_web.py)."""
    bundler = _load(GS_ROOT / "tools" / "bundle_web.py")
    assert bundler.OUT.read_text(encoding="utf-8") == bundler.build()
