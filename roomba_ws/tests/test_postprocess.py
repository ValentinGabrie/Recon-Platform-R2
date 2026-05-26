"""Unit tests for the Tier 2 post-processing pipeline.

Covers each stage in isolation plus the full top-level call:

  * median_3x3 removes a single stray cell
  * morphological_opening removes specks
  * morphological_closing fills a 1-cell gap in a wall
  * label_connected_components labels each blob distinctly
  * process_grid is idempotent on a clean map
  * process_grid drops sub-min_cluster blobs into noise
"""

import numpy as np
import pytest

from recon_db.postprocess import (
    hough_line_segments,
    label_connected_components,
    median_3x3,
    morphological_closing,
    morphological_opening,
    process_grid,
    ALGORITHM_NAME,
)


def _grid(rows: list[str], symbols: dict[str, int]) -> np.ndarray:
    """Compact way to build a test grid from ASCII art."""
    return np.array([[symbols[ch] for ch in row] for row in rows], dtype=np.int8)


# ---------------------------------------------------------------------------
# Stage 1 — median
# ---------------------------------------------------------------------------

def test_median_removes_single_stray_cell():
    """median_3x3 is opt-in (not in the default pipeline) but still has
    to do its job when called directly — a single stray wall surrounded
    by free cells must vanish."""
    SYMS = {".": 0, "#": 100}
    inp = _grid([
        "......",
        "..#...",   # one stray "wall" surrounded by free cells
        "......",
        "......",
    ], SYMS)
    out = median_3x3(inp)
    assert out[1, 2] == 0


def test_median_preserves_dense_walls():
    SYMS = {".": 0, "#": 100}
    inp = _grid([
        "######",
        "######",
        "######",
        "######",
    ], SYMS)
    out = median_3x3(inp)
    assert np.array_equal(out, inp)


# ---------------------------------------------------------------------------
# Stage 2/3 — morphological operations
# ---------------------------------------------------------------------------

def test_opening_removes_speck():
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 2] = True   # single isolated speck
    out = morphological_opening(mask)
    assert not out.any(), f"speck should be gone, got {out.sum()} True cells"


def test_closing_fills_one_cell_gap_in_wall():
    # 5-cell wall with a single missing cell in the middle.
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, :] = True
    mask[2, 2] = False   # the gap
    out = morphological_closing(mask)
    assert out[2, 2], "1-cell gap in a wall should be closed"


def test_opening_destroys_thin_walls_by_design():
    """Document the morphology gotcha: opening with iters≥1 will erode
    walls thinner than 3 cells. That's why DEFAULT_PARAMS sets
    opening_iterations=0 — closing alone keeps thin walls intact.
    """
    mask = np.zeros((6, 6), dtype=bool)
    mask[3, :] = True   # 1-cell-thick wall
    out = morphological_opening(mask, iterations=1)
    assert not out.any(), "1-cell-thick walls should be eroded by opening — known + intentional"


def test_opening_then_closing_idempotent_on_thick_interior_block():
    """3-cell-thick block fully surrounded by free space — round-trip = no-op.

    Closing isn't idempotent in general (it grows along grid edges), but on
    an interior block padded by ≥ 1 free cell, opening-then-closing is.
    """
    mask = np.zeros((8, 8), dtype=bool)
    mask[3:6, 2:6] = True   # 3-row solid block, free margin on all sides
    out = morphological_closing(morphological_opening(mask))
    assert np.array_equal(out, mask)


# ---------------------------------------------------------------------------
# Stage 4 — connected components
# ---------------------------------------------------------------------------

def test_components_two_disjoint_blobs():
    mask = np.zeros((5, 9), dtype=bool)
    mask[1:4, 0:3] = True    # blob A
    mask[1:4, 6:9] = True    # blob B
    labels, n = label_connected_components(mask)
    assert n == 2
    a_label = labels[2, 1]
    b_label = labels[2, 7]
    assert a_label != b_label
    assert (labels == a_label).sum() == 9
    assert (labels == b_label).sum() == 9
    assert (labels == -1).sum() == mask.size - 18


def test_components_8_connected_diagonal():
    # Two cells touching only diagonally — 8-connectivity says one blob.
    mask = np.zeros((4, 4), dtype=bool)
    mask[1, 1] = True
    mask[2, 2] = True
    _, n = label_connected_components(mask)
    assert n == 1


# ---------------------------------------------------------------------------
# Top-level process_grid
# ---------------------------------------------------------------------------

def _flat(arr: np.ndarray) -> list[int]:
    return arr.flatten().astype(int).tolist()


def test_process_grid_clears_a_lonely_speck():
    SYMS = {".": 0, "#": 100, "?": -1}
    inp = _grid([
        "..........",
        "....#.....",   # one stray wall cell
        "..........",
        "..........",
    ], SYMS)
    result = process_grid(_flat(inp), inp.shape[1], inp.shape[0],
                          params={"min_cluster_size": 4})
    # The speck (size 1) is below min_cluster_size and must be cleared.
    assert result.n_clusters == 0
    assert result.cleaned[1, 4] == 0


def test_process_grid_keeps_a_solid_wall_as_one_cluster():
    SYMS = {".": 0, "#": 100}
    # 2-row wall flanked by a free margin on all sides — matches what
    # slam_toolbox actually produces (it always pads with free border).
    inp = _grid([
        "............",
        "............",
        ".##########.",
        ".##########.",
        "............",
        "............",
    ], SYMS)
    result = process_grid(_flat(inp), inp.shape[1], inp.shape[0],
                          params={"min_cluster_size": 4})
    assert result.n_clusters == 1
    assert (result.cluster_labels == 0).sum() == 20


def test_process_grid_separates_two_objects():
    SYMS = {".": 0, "#": 100}
    inp = _grid([
        "........................",
        "..####..........####....",   # two distinct wall blocks
        "..####..........####....",
        "..####..........####....",
        "........................",
    ], SYMS)
    result = process_grid(_flat(inp), inp.shape[1], inp.shape[0],
                          params={"min_cluster_size": 4})
    assert result.n_clusters == 2


def test_process_grid_round_trip_to_json():
    SYMS = {".": 0, "#": 100}
    inp = _grid([
        "....",
        ".##.",
        ".##.",
        "....",
    ], SYMS)
    result = process_grid(_flat(inp), 4, 4)
    blob = result.to_json_bytes()
    import json as _json
    payload = _json.loads(blob)
    assert payload["width"] == 4
    assert payload["height"] == 4
    assert payload["n_clusters"] == result.n_clusters
    assert payload["algorithm"] == ALGORITHM_NAME
    assert len(payload["data"]) == 16
    assert len(payload["cluster_labels"]) == 16


def test_process_grid_preserves_unknown_cells():
    """Unknown (-1) cells should remain unknown after processing."""
    SYMS = {".": 0, "#": 100, "?": -1}
    inp = _grid([
        "?????????.",
        "?####?????",
        "?####?????",
        "?????????.",
    ], SYMS)
    result = process_grid(_flat(inp), inp.shape[1], inp.shape[0],
                          params={"min_cluster_size": 2})
    # Top-left corner unknown stays unknown.
    assert result.cleaned[0, 0] == -1


# ---------------------------------------------------------------------------
# Stage 5 — Hough Line Transform
# ---------------------------------------------------------------------------

def _segment_axis(seg, axis):
    """Return the bounding-box span of a segment along axis 0 (y) or 1 (x)."""
    if axis == 1:
        return abs(seg[2] - seg[0])
    return abs(seg[3] - seg[1])


def test_hough_finds_horizontal_line():
    mask = np.zeros((20, 30), dtype=bool)
    mask[10, 2:28] = True   # 26-cell horizontal segment
    segs = hough_line_segments(mask, vote_thresh=10, min_len=8)
    assert len(segs) >= 1, "should detect the horizontal line"
    # At least one segment must run mostly along x (Δx >> Δy)
    longest = max(segs, key=lambda s: _segment_axis(s, 1))
    assert _segment_axis(longest, 1) >= 20, f"longest x-span too small: {longest}"
    assert _segment_axis(longest, 0) <= 2,  f"horizontal line shouldn't have y-span: {longest}"


def test_hough_finds_vertical_line():
    mask = np.zeros((30, 20), dtype=bool)
    mask[2:28, 10] = True   # 26-cell vertical segment
    segs = hough_line_segments(mask, vote_thresh=10, min_len=8)
    assert len(segs) >= 1
    longest = max(segs, key=lambda s: _segment_axis(s, 0))
    assert _segment_axis(longest, 0) >= 20
    assert _segment_axis(longest, 1) <= 2


def test_hough_finds_diagonal_line():
    mask = np.zeros((30, 30), dtype=bool)
    for i in range(25):
        mask[2 + i, 2 + i] = True   # main diagonal
    segs = hough_line_segments(mask, vote_thresh=10, min_len=8)
    assert len(segs) >= 1
    # Diagonal: |Δx| ≈ |Δy|, both substantial
    longest = max(segs, key=lambda s: _segment_axis(s, 1) + _segment_axis(s, 0))
    dx = _segment_axis(longest, 1)
    dy = _segment_axis(longest, 0)
    assert dx >= 10 and dy >= 10
    assert abs(dx - dy) <= 4   # within a few pixels of equal


def test_hough_ignores_noise():
    """Sparse random pixels should produce zero segments (no line passes the vote threshold)."""
    rng = np.random.default_rng(42)
    mask = rng.random((40, 40)) < 0.03   # ~3% density, no collinear structure
    segs = hough_line_segments(mask, vote_thresh=20, min_len=12)
    assert segs == [], f"expected no segments from random noise, got {segs}"


def test_hough_finds_room_walls():
    """A simple rectangular room — should yield 4 wall segments (give or take a couple from NMS)."""
    mask = np.zeros((30, 40), dtype=bool)
    mask[5,  5:35] = True    # top wall
    mask[24, 5:35] = True    # bottom wall
    mask[5:25, 5]  = True    # left wall
    mask[5:25, 34] = True    # right wall
    segs = hough_line_segments(mask, vote_thresh=15, min_len=15)
    # Each side should yield at least one segment. Allow some duplicates from
    # nearby Hough peaks; require at least one horizontal + one vertical.
    horizontals = [s for s in segs if _segment_axis(s, 1) > _segment_axis(s, 0) * 3]
    verticals   = [s for s in segs if _segment_axis(s, 0) > _segment_axis(s, 1) * 3]
    assert len(horizontals) >= 2, f"need both horizontal walls; got {horizontals}"
    assert len(verticals)   >= 2, f"need both vertical walls; got {verticals}"


def test_hough_empty_mask_returns_empty():
    mask = np.zeros((10, 10), dtype=bool)
    assert hough_line_segments(mask) == []


def test_process_grid_emits_line_segments_in_json():
    """End-to-end: process_grid should fill in line_segments for a clearly-linear input."""
    SYMS = {".": 0, "#": 100}
    inp = _grid([
        "...................",
        "...................",
        "..###############..",
        "...................",
        "...................",
    ], SYMS)
    result = process_grid(_flat(inp), inp.shape[1], inp.shape[0],
                          params={"min_cluster_size": 4, "hough_vote_thresh": 10,
                                  "hough_min_len": 8})
    assert len(result.line_segments) >= 1
    payload = __import__("json").loads(result.to_json_bytes())
    assert payload["n_lines"] == len(result.line_segments)
    assert payload["line_segments"] == [list(s) for s in result.line_segments]
