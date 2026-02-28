from inspection_manager.alignment import AlignmentController


def test_alignment_simulated_converges():
    align = AlignmentController()
    assert align.converge_until(5.0, 2.0) is True

