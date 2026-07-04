"""Unit tests for the Tier 2 post-processing pipeline.

Covers each stage in isolation plus the full top-level call:

  * median_3x3 removes a single stray cell
  * morphological_opening removes specks
  * morphological_closing fills a 1-cell gap in a wall
  * label_connected_components labels each blob distinctly
  * process_grid is idempotent on a clean map
  * process_grid drops sub-min_cluster blobs into noise
"""

import math

import numpy as np
import pytest

from recon_db.postprocess import (
    estimate_deskew_angle,
    hough_line_segments,
    label_connected_components,
    median_3x3,
    merge_collinear_segments,
    morphological_closing,
    morphological_opening,
    orthogonal_snap,
    process_grid,
    rasterize_segments,
    room_metrics,
    rotate_grid_nn,
    snap_segments,
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


# ---------------------------------------------------------------------------
# Stage 6 — Manhattan-world regularisation (deskew + snap)
# ---------------------------------------------------------------------------

def _tilted_room(size: int = 70, tilt_deg: float = 0.0) -> np.ndarray:
    """Rasterise a rectangular room outline rotated by `tilt_deg` into a grid
    of int8 occupancy (100 = wall, 0 = free)."""
    import math
    cx = cy = size // 2
    pts: list[tuple[int, int]] = []
    for t in range(-20, 21):          # top + bottom walls (along local x)
        pts += [(t, -14), (t, 14)]
    for t in range(-14, 15):          # left + right walls (along local y)
        pts += [(-20, t), (20, t)]
    th = math.radians(tilt_deg)
    c, s = math.cos(th), math.sin(th)
    grid = np.zeros((size, size), dtype=np.int8)
    for x, y in pts:
        gx = int(round(cx + c * x - s * y))
        gy = int(round(cy + s * x + c * y))
        if 0 <= gx < size and 0 <= gy < size:
            grid[gy, gx] = 100
    return grid


def test_estimate_deskew_axis_aligned_is_near_zero():
    room = _tilted_room(tilt_deg=0.0) >= 50
    assert abs(np.degrees(estimate_deskew_angle(room))) < 1.0


def test_estimate_deskew_empty_mask_is_zero():
    assert estimate_deskew_angle(np.zeros((10, 10), dtype=bool)) == 0.0


def test_estimate_deskew_no_structure_is_zero():
    # Sparse noise has no dominant orientation → no rotation.
    rng = np.random.default_rng(1)
    mask = rng.random((40, 40)) < 0.02
    assert estimate_deskew_angle(mask) == 0.0


def test_deskew_straightens_a_tilted_room():
    """estimate → rotate should leave the room axis-aligned regardless of the
    sign convention: re-estimating on the rotated grid gives ~0."""
    for tilt in (8.0, 13.0, -11.0):
        room = _tilted_room(tilt_deg=tilt) >= 50
        angle = estimate_deskew_angle(room)
        straight = rotate_grid_nn(room.astype(np.int8), angle, fill=0) >= 50
        residual = abs(np.degrees(estimate_deskew_angle(straight)))
        assert residual < 2.0, f"tilt={tilt} left residual {residual:.1f}°"


def test_deskew_skips_diffuse_orientation():
    """The b020 lesson: walls spread across many angles (drift-smeared, multi-
    room) have low angular concentration → no rotation, so the deskew can't
    degrade a messy map."""
    import math
    size = 90
    g = np.zeros((size, size), dtype=np.int8)
    cx = cy = size // 2
    for deg in (0, 20, 40, 60, 80, 110, 140):     # 7 walls, no common axis
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        for t in range(-28, 29):
            x, y = int(round(cx + t * c)), int(round(cy + t * s))
            if 0 <= x < size and 0 <= y < size:
                g[y, x] = 100
    assert estimate_deskew_angle(g >= 50) == 0.0


def test_deskew_concentration_threshold_param():
    room = _tilted_room(tilt_deg=12.0) >= 50
    # A too-strict concentration floor rejects even a clean tilted room.
    assert estimate_deskew_angle(room, min_concentration=0.9) == 0.0
    # The default-ish floor accepts it.
    assert abs(np.degrees(estimate_deskew_angle(room, min_concentration=0.2))) > 5.0


def test_rotate_zero_is_identity():
    grid = _tilted_room(tilt_deg=0.0)
    out = rotate_grid_nn(grid, 0.0, fill=-1)
    assert np.array_equal(out, grid)


def test_snap_near_horizontal_becomes_flat():
    out = snap_segments([(0, 0, 20, 1)], snap_deg=8.0)
    assert out == [(0, 0, 20, 0)]


def test_snap_near_vertical_becomes_plumb():
    out = snap_segments([(0, 0, 1, 20)], snap_deg=8.0)
    assert out == [(0, 0, 0, 20)]


def test_snap_leaves_true_diagonal_alone():
    seg = (0, 0, 20, 20)
    assert snap_segments([seg], snap_deg=8.0) == [seg]


def test_process_grid_deskews_a_tilted_map():
    tilted = _tilted_room(tilt_deg=12.0)
    result = process_grid(_flat(tilted), tilted.shape[1], tilted.shape[0],
                          params={"min_cluster_size": 8})
    # A 12° tilt is well past the 0.75° min, so the map must be rotated.
    assert abs(result.deskew_deg) > 5.0
    # And the rotated result should be essentially axis-aligned now.
    occ = result.cleaned >= 50
    assert abs(np.degrees(estimate_deskew_angle(occ))) < 2.0
    payload = __import__("json").loads(result.to_json_bytes())
    assert payload["deskew_deg"] == round(result.deskew_deg, 2)


def test_process_grid_already_aligned_is_not_rotated():
    """Regression guard: a straight map must stay pixel-identical (no lossy
    resample) and report deskew_deg == 0."""
    room = _tilted_room(tilt_deg=0.0)
    result = process_grid(_flat(room), room.shape[1], room.shape[0],
                          params={"min_cluster_size": 8})
    assert result.deskew_deg == 0.0
    assert result.cleaned.shape == room.shape


def test_manhattan_align_can_be_disabled():
    tilted = _tilted_room(tilt_deg=12.0)
    result = process_grid(_flat(tilted), tilted.shape[1], tilted.shape[0],
                          params={"min_cluster_size": 8, "manhattan_align": 0})
    assert result.deskew_deg == 0.0
    assert result.cleaned.shape == tilted.shape


# ---------------------------------------------------------------------------
# Stage 5b — Hough fill-ratio gate
# ---------------------------------------------------------------------------

def test_hough_fill_gate_rejects_sparse_chord():
    """A row of cells spaced every 3rd column is a chord through mostly-empty
    space (fill ≈ 0.35), not a wall. Default min_fill=0 keeps the old
    behaviour; min_fill=0.5 must reject it."""
    mask = np.zeros((20, 40), dtype=bool)
    mask[10, 2:38:3] = True   # 12 cells over a 34-cell span → fill ≈ 0.35
    kept = hough_line_segments(mask, vote_thresh=10, min_len=8, max_gap=3)
    assert len(kept) >= 1, "without the gate the stitched chord is emitted"
    gated = hough_line_segments(mask, vote_thresh=10, min_len=8, max_gap=3,
                                min_fill=0.5)
    assert gated == [], f"sparse chord should be gated out, got {gated}"


def test_hough_fill_gate_keeps_solid_wall():
    mask = np.zeros((20, 40), dtype=bool)
    mask[10, 2:38] = True      # solid → fill ≈ 1.0
    gated = hough_line_segments(mask, vote_thresh=10, min_len=8, min_fill=0.5)
    assert len(gated) >= 1


# ---------------------------------------------------------------------------
# Stage 7 — collinear segment merge (wall de-duplication)
# ---------------------------------------------------------------------------

def test_merge_passthrough_for_trivial_input():
    assert merge_collinear_segments([]) == []
    assert merge_collinear_segments([(0, 0, 5, 5)]) == [(0, 0, 5, 5)]


def test_merge_collapses_duplicate_walls():
    """The Hough 'bundle' — several near-identical segments for one wall —
    must collapse to a single wall spanning the full extent."""
    bundle = [(0, 10, 30, 10), (0, 11, 30, 11), (0, 9, 30, 9), (2, 10, 28, 10)]
    out = merge_collinear_segments(bundle, angle_deg=7, offset=3, gap=8)
    assert len(out) == 1
    x0, y0, x1, y1 = out[0]
    assert abs(y0 - 10) <= 1 and abs(y1 - 10) <= 1
    assert min(x0, x1) <= 1 and max(x0, x1) >= 29


def test_merge_keeps_distinct_parallel_walls():
    """Two parallel walls farther apart than `offset` are different walls."""
    out = merge_collinear_segments([(0, 5, 30, 5), (0, 25, 30, 25)], offset=3)
    assert len(out) == 2


def test_merge_bridges_small_collinear_gap():
    """Two collinear fragments with a gap ≤ `gap` join into one wall."""
    out = merge_collinear_segments([(0, 10, 12, 10), (15, 10, 30, 10)], gap=8)
    assert len(out) == 1
    x0, _, x1, _ = out[0]
    assert min(x0, x1) <= 0 and max(x0, x1) >= 30


def test_merge_keeps_large_gap_split():
    """Collinear fragments separated by more than `gap` stay as two walls
    (a doorway is not bridged into a phantom wall)."""
    out = merge_collinear_segments([(0, 10, 10, 10), (25, 10, 35, 10)], gap=8)
    assert len(out) == 2


def test_merge_keeps_perpendicular_walls_separate():
    out = merge_collinear_segments([(0, 10, 30, 10), (15, 0, 15, 30)])
    assert len(out) == 2


def test_merge_output_sorted_longest_first():
    out = merge_collinear_segments([(0, 0, 5, 0), (0, 20, 40, 20)])
    lengths = [(s[2] - s[0]) ** 2 + (s[3] - s[1]) ** 2 for s in out]
    assert lengths == sorted(lengths, reverse=True)


def test_process_grid_collapses_wall_bundle():
    """Regression for the 'unreal number of walls' bug: a rectangular room
    with a noisy boundary + salt noise must emit a handful of clean,
    axis-aligned walls — not the dozens the raw Hough pass produced."""
    import math
    H, W = 60, 44
    g = np.zeros((H, W), dtype=np.int8)
    g[6, 5:39] = 100       # top wall
    g[53, 5:39] = 100      # bottom wall
    g[6:54, 5] = 100       # left wall
    g[6:54, 38] = 100      # right wall
    for (y, x) in [(20, 15), (33, 27), (41, 12), (15, 30), (48, 22), (25, 8)]:
        g[y, x] = 100      # scattered salt noise
    result = process_grid(g.flatten().astype(int).tolist(), W, H,
                          params={"min_cluster_size": 4})
    segs = result.line_segments
    assert 1 <= len(segs) <= 8, f"expected a handful of walls, got {len(segs)}: {segs}"

    def _ang(s):
        return math.degrees(math.atan2(s[3] - s[1], s[2] - s[0])) % 180.0
    horiz = [s for s in segs if min(_ang(s), 180 - _ang(s)) <= 10]
    vert = [s for s in segs if abs(_ang(s) - 90) <= 10]
    assert horiz and vert, f"room needs H and V walls: {[round(_ang(s)) for s in segs]}"


def test_max_walls_caps_output():
    """A spray of unrelated short segments is capped to max_walls (longest kept)."""
    segs = [(0, 4 * i, 6 + i, 4 * i) for i in range(12)]   # 12 parallel-ish stubs
    result_segs = merge_collinear_segments(segs, offset=1, gap=1)
    assert len(result_segs) >= 1
    # The cap itself is exercised through process_grid params below.
    H, W = 50, 50
    g = np.zeros((H, W), dtype=np.int8)
    for i in range(12):
        g[2 + 4 * i % 48, 2:48] = 100   # several stacked horizontal walls
    res = process_grid(g.flatten().astype(int).tolist(), W, H,
                       params={"min_cluster_size": 2, "max_walls": 3})
    assert len(res.line_segments) <= 3


# ---------------------------------------------------------------------------
# Stage 8 — wall baking
# ---------------------------------------------------------------------------

def test_rasterize_horizontal_segment():
    m = rasterize_segments([(2, 10, 30, 10)], (20, 40), thickness=1)
    assert m[10, 2] and m[10, 16] and m[10, 30]
    assert not m[9, 16] and not m[11, 16]


def test_rasterize_thickness_widens():
    m1 = rasterize_segments([(2, 10, 30, 10)], (20, 40), thickness=1)
    m2 = rasterize_segments([(2, 10, 30, 10)], (20, 40), thickness=2)
    assert m2.sum() > m1.sum()


def test_rasterize_clips_to_grid():
    m = rasterize_segments([(-5, 5, 100, 5)], (10, 20), thickness=1)
    assert m.shape == (10, 20)
    assert m[5, :].all()       # the in-bounds part of the row is filled


def test_process_grid_bakes_uniform_walls():
    """Baking on: the cleaned grid's walls become exactly the rasterised
    detected segments (1-cell uniform), not the ragged input boundary."""
    H, W = 40, 30
    g = np.full((H, W), -1, dtype=np.int8)
    g[4:36, 3:27] = 0          # free interior
    g[4, 3:27] = 100           # top
    g[35, 3:27] = 100          # bottom
    g[4:36, 3] = 100           # left
    g[4:36, 26] = 100          # right
    res = process_grid(g.flatten().astype(int).tolist(), W, H,
                       params={"resolution": 0.05, "min_cluster_size": 4})
    assert len(res.line_segments) >= 1
    baked = (res.cleaned >= 50)
    assert baked.sum() > 0
    # Every baked wall cell must lie on a detected segment's raster.
    expected = rasterize_segments(res.line_segments, res.cleaned.shape,
                                  thickness=int(res.parameters["wall_thickness"]))
    assert np.array_equal(baked, expected)


def test_process_grid_bake_can_be_disabled():
    """Baking replaces thick/ragged walls with uniform 1-cell walls, so it
    strictly reduces the wall-cell count vs leaving the raw boundary in place."""
    H, W = 50, 36
    g = np.full((H, W), -1, dtype=np.int8)
    g[5:45, 4:32] = 0
    g[5:8, 4:32] = 100      # 3-cell-thick top wall
    g[42:45, 4:32] = 100    # 3-cell-thick bottom wall
    g[5:45, 4:7] = 100      # 3-cell-thick left wall
    g[5:45, 29:32] = 100    # 3-cell-thick right wall
    flat = g.flatten().astype(int).tolist()
    baked = process_grid(flat, W, H,
                         params={"min_cluster_size": 4, "bake_walls": 1, "wall_thickness": 1})
    raw = process_grid(flat, W, H, params={"min_cluster_size": 4, "bake_walls": 0})
    assert (baked.cleaned >= 50).sum() < (raw.cleaned >= 50).sum()
    # Room metrics are reported either way.
    assert baked.room["enclosed"] == raw.room["enclosed"]


# ---------------------------------------------------------------------------
# Stage 8 — room metrics + enclosure detection
# ---------------------------------------------------------------------------

def _sealed_room(H=30, W=40, res_free=True):
    """Unknown background, a free interior, walls sealing it — a real room."""
    g = np.full((H, W), -1, dtype=np.int8)
    g[5:25, 5:35] = 0
    g[5, 5:35] = 100
    g[24, 5:35] = 100
    g[5:25, 5] = 100
    g[5:25, 34] = 100
    return g


def test_room_metrics_detects_enclosed_room():
    g = _sealed_room()
    m = room_metrics(g, occupied_threshold=50, resolution=0.05, seal=0)
    assert m["enclosed"] is True
    # bbox of walls: 20 rows × 30 cols → 1.0 m × 1.5 m at 5 cm/cell
    assert m["length_m"] == 1.5
    assert m["width_m"] == 1.0
    assert m["area_m2"] > 0.0


def test_room_metrics_open_scan_not_enclosed():
    g = _sealed_room()
    g[24, 5:35] = 0    # remove the whole bottom wall → interior leaks out
    m = room_metrics(g, occupied_threshold=50, resolution=0.05, seal=2)
    assert m["enclosed"] is False


def test_room_metrics_seal_closes_doorway():
    g = _sealed_room()
    g[24, 18:22] = 0   # a 4-cell doorway in the bottom wall
    open_eval = room_metrics(g, occupied_threshold=50, resolution=0.05, seal=0)
    sealed_eval = room_metrics(g, occupied_threshold=50, resolution=0.05, seal=3)
    assert open_eval["enclosed"] is False     # the gap leaks
    assert sealed_eval["enclosed"] is True     # sealing bridges the doorway


def test_room_metrics_empty_grid_is_zero():
    g = np.full((10, 10), -1, dtype=np.int8)
    m = room_metrics(g, occupied_threshold=50, resolution=0.05)
    assert m == {"enclosed": False, "length_m": 0.0, "width_m": 0.0, "area_m2": 0.0}


def test_room_metrics_scales_with_resolution():
    g = _sealed_room()
    m1 = room_metrics(g, 50, 0.05, seal=0)
    m2 = room_metrics(g, 50, 0.10, seal=0)
    assert abs(m2["length_m"] - 2 * m1["length_m"]) < 1e-6
    assert abs(m2["area_m2"] - 4 * m1["area_m2"]) < 1e-6


def test_process_grid_room_in_json_payload():
    g = _sealed_room()
    res = process_grid(g.flatten().astype(int).tolist(), g.shape[1], g.shape[0],
                       params={"resolution": 0.05, "min_cluster_size": 4})
    payload = __import__("json").loads(res.to_json_bytes())
    assert payload["room"] == res.room
    assert set(res.room) == {"enclosed", "length_m", "width_m", "area_m2"}


# ---------------------------------------------------------------------------
# Stage 6b — orthogonal snap (square corners)
# ---------------------------------------------------------------------------

def _angle180(seg):
    return math.degrees(math.atan2(seg[3] - seg[1], seg[2] - seg[0])) % 180.0


def test_ortho_snap_passthrough():
    assert orthogonal_snap([], 12.0) == []
    assert orthogonal_snap([(0, 0, 5, 5)], 0) == [(0, 0, 5, 5)]


def test_ortho_snap_squares_a_corner():
    """Two walls meeting at ~86° are squared to ~90° apart. (The residual is
    integer-endpoint quantisation — a 40-cell wall rounds to ≈1.4°/cell.)"""
    segs = [(0, 0, 40, 0), (0, 0, 3, 40)]   # ~0° and ~85.7°
    out = orthogonal_snap(segs, snap_deg=12.0)
    diff = abs(_angle180(out[0]) - _angle180(out[1]))
    diff = min(diff, 180 - diff)
    assert abs(diff - 90) < 3.0, f"corner not square: {diff:.2f}°"


def test_ortho_snap_works_at_global_tilt():
    """Walls scattered around a 10°-tilted orthogonal grid collapse onto it:
    two parallel + two perpendicular, exactly 90° apart — no deskew needed."""
    def seg_at(deg, length=40, cx=50, cy=50):
        th = math.radians(deg)
        hx, hy = length / 2 * math.cos(th), length / 2 * math.sin(th)
        return (int(cx - hx), int(cy - hy), int(cx + hx), int(cy + hy))
    segs = [seg_at(9), seg_at(11), seg_at(98), seg_at(102)]
    out = orthogonal_snap(segs, snap_deg=12.0)
    angs = sorted(_angle180(s) for s in out)
    assert abs(angs[0] - angs[1]) < 1.0          # the two ~10° walls coincide
    assert abs(angs[2] - angs[3]) < 1.0          # the two ~100° walls coincide
    assert abs((angs[2] - angs[0]) - 90) < 1.0   # families exactly 90° apart


def test_ortho_snap_leaves_odd_angle_alone():
    """A genuine 45° diagonal (>snap_deg off the grid) is untouched."""
    segs = [(0, 0, 40, 0), (0, 10, 40, 10), (0, 0, 0, 40),
            (20, 0, 20, 40), (0, 0, 28, 28)]
    out = orthogonal_snap(segs, snap_deg=12.0)
    assert out[4] == (0, 0, 28, 28)


def test_ortho_snap_preserves_length():
    segs = [(0, 0, 40, 0), (0, 0, 4, 40)]
    out = orthogonal_snap(segs, snap_deg=12.0)
    for si, so in zip(segs, out):
        li = math.hypot(si[2] - si[0], si[3] - si[1])
        lo = math.hypot(so[2] - so[0], so[3] - so[1])
        assert abs(li - lo) <= 1.5


def test_process_grid_squares_corners_without_deskew():
    """Even with the Manhattan deskew disabled, a tilted rectangular room comes
    out with every wall parallel or perpendicular to the longest wall."""
    tilted = _tilted_room(tilt_deg=7.0)
    res = process_grid(_flat(tilted), tilted.shape[1], tilted.shape[0],
                       params={"manhattan_align": 0, "min_cluster_size": 8})
    assert res.deskew_deg == 0.0
    segs = res.line_segments
    assert len(segs) >= 2
    ref = max(segs, key=lambda s: math.hypot(s[2] - s[0], s[3] - s[1]))
    ref_a = math.atan2(ref[3] - ref[1], ref[2] - ref[0])
    for s in segs:
        a = math.atan2(s[3] - s[1], s[2] - s[0])
        d = (a - ref_a) % (math.pi / 2)
        d = min(d, math.pi / 2 - d)
        assert math.degrees(d) < 4.0, f"wall off the orthogonal grid by {math.degrees(d):.1f}°"


# ---------------------------------------------------------------------------
# Stage 4.5 — free-space opening (ray-fan removal)
# ---------------------------------------------------------------------------

def _room_with_ray_fan(H=60, W=80):
    """A sealed room plus thin 1-cell 'free' rays leaking through a gap —
    the streak fan a handheld scan carves through windows / door gaps."""
    g = np.full((H, W), -1, dtype=np.int8)
    g[10:40, 10:50] = 0            # free interior
    g[10, 10:50] = 100             # walls
    g[39, 10:50] = 100
    g[10:40, 10] = 100
    g[10:40, 49] = 100
    for i, y in enumerate(range(12, 38, 3)):   # thin rays outside the room
        for x in range(50, 78):
            g[y + (i % 2), x] = 0
    return g


def test_free_opening_dissolves_ray_fan():
    g = _room_with_ray_fan()
    res = process_grid(_flat(g), g.shape[1], g.shape[0],
                       params={"min_cluster_size": 4, "bake_walls": 0,
                               "manhattan_align": 0})
    # Every 1-cell-thin free ray outside the room must revert to unknown.
    assert (res.cleaned[:, 55:] == 0).sum() == 0
    # The solid interior survives intact.
    assert (res.cleaned[12:38, 12:48] == 0).all()


def test_free_opening_can_be_disabled():
    g = _room_with_ray_fan()
    res = process_grid(_flat(g), g.shape[1], g.shape[0],
                       params={"min_cluster_size": 4, "bake_walls": 0,
                               "manhattan_align": 0,
                               "free_opening_iterations": 0})
    assert (res.cleaned[:, 55:] == 0).sum() > 0    # rays kept verbatim


# ---------------------------------------------------------------------------
# Stage 6 — segment-based tilt estimate (deskew that fires on smeared maps)
# ---------------------------------------------------------------------------

def test_estimate_tilt_from_segments_rectilinear():
    from recon_db.postprocess import estimate_tilt_from_segments
    # Two perpendicular wall families tilted 20° off-axis.
    segs = [(0, 0, 94, 34), (10, 60, 104, 94), (0, 0, 34, -94)]
    angle, conc, total = estimate_tilt_from_segments(segs)
    assert conc > 0.95
    assert total > 100
    assert abs(math.degrees(angle) + 20.0) < 2.0   # rotation to APPLY is -20°


def test_estimate_tilt_from_segments_empty():
    from recon_db.postprocess import estimate_tilt_from_segments
    assert estimate_tilt_from_segments([]) == (0.0, 0.0, 0.0)


def test_process_grid_deskews_a_smeared_tilted_map():
    """Regression: the accumulator-concentration gate left the deskew dormant
    on real handheld maps because wall smear diluted it below 0.2. The
    segment-based estimate must still fire when the walls are 3 cells thick
    and noisy."""
    rng = np.random.default_rng(7)
    base = _tilted_room(size=90, tilt_deg=18.0)
    occ = base >= 50
    smear = occ.copy()
    for _ in range(2):                     # thicken walls to a 3-cell smear
        from recon_db.postprocess import _binary_dilate
        smear = _binary_dilate(smear)
    g = np.where(smear, 100, base).astype(np.int8)
    noise_y = rng.integers(0, 90, 40)
    noise_x = rng.integers(0, 90, 40)
    g[noise_y, noise_x] = 100              # salt noise
    res = process_grid(_flat(g), g.shape[1], g.shape[0],
                       params={"min_cluster_size": 8})
    assert abs(res.deskew_deg) > 10.0, "deskew must fire on a smeared tilted room"


# ---------------------------------------------------------------------------
# Stage 8 — conservative bake (keep structure the detector missed)
# ---------------------------------------------------------------------------

def test_bake_keeps_unmatched_obstacle():
    """An obstacle far from every detected wall (e.g. furniture) must survive
    baking as occupied — the old clear-everything bake turned it into free
    floor, fabricating open space."""
    H, W = 60, 80
    g = np.full((H, W), -1, dtype=np.int8)
    g[5:55, 5:75] = 0
    g[5, 5:75] = 100
    g[54, 5:75] = 100
    g[5:55, 5] = 100
    g[5:55, 74] = 100
    g[28:33, 38:43] = 100          # 5×5 obstacle in the middle of the room
    res = process_grid(_flat(g), W, H,
                       params={"min_cluster_size": 4, "manhattan_align": 0})
    assert (res.cleaned[28:33, 38:43] >= 50).any(), \
        "isolated obstacle was erased by baking"


def test_bake_absorbs_ragged_wall_into_straight_wall():
    """Ragged occupied cells hugging a detected wall ARE absorbed (that's the
    point of baking) — only far-from-wall structure is preserved."""
    H, W = 50, 60
    g = np.full((H, W), -1, dtype=np.int8)
    g[5:45, 5:55] = 0
    g[5:8, 5:55] = 100             # 3-cell-thick smeared top wall
    g[44, 5:55] = 100
    g[5:45, 5] = 100
    g[5:45, 54] = 100
    res = process_grid(_flat(g), W, H,
                       params={"min_cluster_size": 4, "manhattan_align": 0,
                               "wall_thickness": 1})
    from recon_db.postprocess import rasterize_segments as _rs
    expected = _rs(res.line_segments, res.cleaned.shape, thickness=1)
    baked = res.cleaned >= 50
    # No stray occupied cells beyond the rasterised walls: the smear around
    # each detected wall was absorbed.
    assert np.array_equal(baked, expected)
