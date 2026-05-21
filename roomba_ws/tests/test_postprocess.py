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
