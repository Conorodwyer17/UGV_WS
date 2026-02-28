import importlib.util
from pathlib import Path


_ALIGNMENT_PATH = Path(__file__).resolve().parents[1] / "inspection_manager" / "alignment.py"
_SPEC = importlib.util.spec_from_file_location("canonical_alignment_module", str(_ALIGNMENT_PATH))
_MOD = importlib.util.module_from_spec(_SPEC)
assert _SPEC is not None and _SPEC.loader is not None
_SPEC.loader.exec_module(_MOD)
AlignmentController = _MOD.AlignmentController


def test_alignment_converges_with_good_feedback_stream():
    stream = [
        (0.12, 8.0, 0.7),
        (0.08, 5.0, 0.85),
        (0.049, 2.8, 0.92),
        (0.045, 2.7, 0.95),
        (0.044, 2.5, 0.96),
    ]
    hits = 0
    for p, a, q in stream:
        if AlignmentController.is_converged(p, a, q):
            hits += 1
        else:
            hits = 0
    assert hits >= 3


def test_alignment_rejects_poor_quality_stream():
    stream = [
        (0.03, 2.0, 0.70),
        (0.03, 2.0, 0.75),
        (0.03, 2.0, 0.80),
    ]
    assert all(not AlignmentController.is_converged(p, a, q) for p, a, q in stream)
