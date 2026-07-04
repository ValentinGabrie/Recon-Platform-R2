"""Post-processing pipeline for saved occupancy grids — Tier 2.

Input is a `nav_msgs/OccupancyGrid` flattened into a list of int8 cell
values (see MapRecord.map_data). Output is the cleaned grid plus a
per-cell cluster label so the web UI can colour distinct obstacles.

The pipeline runs on a Raspberry Pi 5 with no scipy / scikit-image /
scikit-learn installed (we want to keep the AP-isolated install lean),
so everything is implemented in pure NumPy. For a 200 × 200 grid this
completes in well under a second.

Stages:

  1. **Median filter (3×3)** — removes salt-and-pepper noise where a
     single stray cell got marked occupied or free in error.

  2. **Morphological opening (3×3)** — erosion followed by dilation
     on the occupied mask. Removes specks but preserves connected
     walls.

  3. **Morphological closing (3×3)** — dilation followed by erosion.
     Closes 1-cell gaps in walls without merging distant features.

  4. **Connected-component labelling (8-connectivity)** with a
     min-cluster filter. This is DBSCAN-equivalent for a binary grid:
     eps = √2 (allow diagonal neighbours), min_samples = min_cluster_size.
     Cells in clusters smaller than ``min_cluster_size`` are reclassified
     as "unknown" (noise) — they're almost always scan-matching artefacts
     or transient obstacles like a person who walked through once.

  4.5 **Free-space opening** — morphological opening on the FREE mask.
     Handheld scans spray long 1–2-cell-wide "free" rays through windows,
     glass and door gaps (the LIDAR gets a distant return, so ray-tracing
     carves a thin free streak far outside the room). On real saved maps
     these fans are 10–17 % of all free cells and are what makes the map
     read as a "cloud". Opening dissolves anything thinner than 3 cells;
     removed cells go back to *unknown* (we have no reliable information
     there). Solid interiors ≥3 cells wide are preserved exactly.

  5. **Hough Line Transform** with a *fill-ratio* gate — emits straight
     wall segments and rejects "lines" that are really a chord stitched
     across mostly-empty space (the classic Hough false positive).

  6. **Manhattan deskew + orthogonal snap** — rotates the dominant wall
     family onto the axes and snaps near-orthogonal segments to exact
     horizontal / vertical. The tilt is estimated from the *merged Hough
     wall segments* (length-weighted circular mean folded mod 90°), NOT
     from the raw accumulator: on real handheld maps the wall smear
     dilutes the accumulator's angular concentration below any usable
     gate (measured 0.16–0.21 on field maps vs the 0.2 threshold, so the
     deskew effectively never fired), while the merged segments give a
     resultant of 0.75–0.99 with the correct angle on the same maps.

  7. **Collinear segment merge** — the Hough accumulator reports the same
     physical wall as a *bundle* of near-duplicate, fragmented segments
     (this is what makes a clean room look like it has 60 walls). This
     stage groups segments by orientation + perpendicular offset and
     unions their spans, collapsing each bundle into one wall. A final
     cap keeps only the longest walls. This is the deterministic,
     geometric counterpart of fuzzy-clustering the wall hypotheses (see
     ``merge_collinear_segments`` for the "where would fuzzy go" note).

The output cluster grid uses ``-1`` for "not occupied" (free or unknown
cells in the cleaned grid) and ``0, 1, 2, …`` for each distinct cluster.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np


ALGORITHM_NAME = "tier2_dbscan_v1"
DEFAULT_PARAMS: dict[str, Any] = {
    # Median defaults to 0 (disabled). A 3×3 median wipes single stray
    # cells but ALSO erodes the edges of walls thinner than ~3 cells,
    # which is bad for typical 5 cm/cell SLAM maps where walls are 1–2
    # cells thick. The min_cluster_size filter below already removes
    # truly isolated noise (a 1-cell component is well under the
    # threshold), so median is redundant for that use case. Enable it
    # (set to 3) on maps from very thick walls if you specifically want
    # mean-style smoothing.
    "median_size":         0,
    # Opening defaults to 0 because 8-connected erosion eats anything
    # thinner than 3 cells. Same reasoning as above — median is unnecessary
    # for noise removal once connected-component filtering is in place.
    "opening_iterations":  0,
    "closing_iterations":  1,    # dilation → erosion passes — fills 1-cell gaps
    "occupied_threshold":  50,   # cells with value ≥ this count as occupied
    "min_cluster_size":    8,    # clusters smaller than this become noise
    # --- Free-space cleanup (ray-fan removal) --------------------------------
    # Opening iterations on the FREE mask. Dissolves the thin "free" rays the
    # LIDAR carves through windows / door gaps (they read as a radial fan of
    # streaks around the room); removed cells revert to unknown. 0 disables.
    # Real rooms are never thinner than 3 cells (15 cm at 5 cm/cell), so the
    # interior is untouched. Opening is idempotent — 1 iteration is enough.
    "free_opening_iterations": 1,
    # --- Hough Line Transform (wall detection) -------------------------------
    # `hough_theta_steps`  number of angle bins from 0..π (1° resolution = 180)
    # `hough_vote_thresh`  minimum accumulator votes for a peak to count as a line
    # `hough_min_len`      minimum on-pixels along the line to emit a segment (cells)
    # `hough_max_gap`      max gap between on-pixels before splitting a segment (cells)
    # `hough_top_n`        emit at most this many strongest *raw* segments (pre-merge)
    # `hough_min_fill`     reject a segment unless this fraction of the cells
    #                      between its endpoints are actually occupied. Kills the
    #                      classic Hough false positive — a long chord stitched
    #                      across scattered cells in open space. A solid wall
    #                      scores ~1.0; a hallucinated diagonal scores < 0.3.
    "hough_theta_steps":   180,
    "hough_vote_thresh":   18,
    "hough_min_len":       8,
    "hough_max_gap":       3,
    "hough_top_n":         60,
    "hough_min_fill":      0.5,
    # --- Collinear segment merge (wall de-duplication) -----------------------
    # The Hough accumulator reports one physical wall as a *bundle* of slightly
    # different (ρ, θ) peaks, so a 4-wall room can emit 30–60 overlapping
    # segments. This stage collapses each bundle into a single wall: segments
    # whose orientation is within `merge_angle_deg` AND whose supporting lines
    # sit within `merge_offset` cells of each other are grouped, then their
    # spans along the wall are unioned (bridging gaps up to `merge_gap` cells).
    # `max_walls` is the final cap — keep only the longest walls so the overlay
    # reads like an architectural plan, not a heatmap.
    "merge_segments":      1,
    "merge_angle_deg":     7.0,
    "merge_offset":        3.0,
    "merge_gap":           8.0,
    "max_walls":           16,
    # --- Manhattan-world regularisation (orientation cleanup) ----------------
    # Handheld scans accumulate yaw drift, so the same wall scanned on two
    # passes lands at slightly different angles and the whole floor plan sits
    # at an arbitrary tilt. `manhattan_align` rotates (deskews) the cleaned
    # grid so the dominant wall direction is axis-aligned — every room then
    # shares one clean orientation. `manhattan_snap_deg` then snaps Hough
    # segments within that many degrees of 0°/90° to exactly horizontal /
    # vertical so the overlay reads like an architectural drawing.
    # `manhattan_min_deg` skips the (lossy) rotation when the map is already
    # within this tolerance of axis-aligned — keeps already-clean maps pixel-
    # identical and avoids needless resampling.
    "manhattan_align":     1,
    "manhattan_snap_deg":  8.0,
    "manhattan_min_deg":   0.75,
    # `ortho_snap` regularises the *whole* wall set onto one orthogonal grid:
    # walls that meet at roughly a right angle are forced to be exactly parallel
    # or perpendicular, so corners come out square. Unlike `manhattan_snap_deg`
    # (which only snaps to the absolute image axes) this works at any global
    # tilt — it finds the dominant direction from the walls themselves and snaps
    # each wall within `ortho_snap_deg` of a 90° multiple of it.
    "ortho_snap":          1,
    "ortho_snap_deg":      12.0,
    # Only deskew when the walls actually share a dominant orientation.
    # Measured on the MERGED Hough segments: length-weighted resultant R
    # (0..1) of the segment angles folded mod 90°. A rectilinear space —
    # even a smeared handheld scan of one — scores 0.75+; a space whose
    # walls point every which way scores well below 0.5. Below the gate we
    # leave the map unrotated rather than inventing an alignment the data
    # doesn't support — deskew never makes a map worse.
    "manhattan_min_concentration": 0.55,
    # Minimum summed length (cells) of the merged wall segments before the
    # tilt estimate is trusted at all — two short strokes always "agree"
    # with each other, that's not evidence of a dominant wall direction.
    "manhattan_min_wall_cells": 40,
    # --- Wall baking + room metrics ------------------------------------------
    # Once the walls are detected and merged we rasterise them back into the
    # grid as clean, uniform straight walls — replacing the ragged scan
    # boundary — so the map reads like a floor plan with no overlay needed
    # (`bake_walls`). `wall_thickness` is the baked wall width in cells.
    # `wall_absorb` bounds how far (in dilation steps beyond the baked wall)
    # ragged occupied cells are considered "explained by" that wall and
    # cleared. Occupied structure FARTHER than that from every detected wall
    # is real evidence the detector missed (furniture, an undetected wall)
    # and is kept as-is — baking must never fabricate open floor where the
    # scan says obstacle, which is exactly what the old clear-everything
    # behaviour did on smeared handheld maps.
    # `resolution` (m/cell) is required for the room dimensions; the API injects
    # the source map's true value (this default is just a 5 cm fallback).
    # `room_seal` dilates the walls by this many cells before the enclosure test
    # so a normal doorway gap doesn't make an otherwise-sealed room read "open".
    "bake_walls":          1,
    "wall_thickness":      3,
    "wall_absorb":         4,
    "resolution":          0.05,
    "room_seal":           2,
}


@dataclass
class ProcessResult:
    cleaned: np.ndarray            # int8 (h, w) — same encoding as input
    cluster_labels: np.ndarray     # int32 (h, w) — -1 = not-occupied / noise
    n_clusters: int
    n_noise_cells: int
    # Hough Line Transform output. Each entry is (x0, y0, x1, y1) in cell-grid
    # coordinates (origin top-left, x right, y down). The web UI overlays these
    # as crisp lines on top of the cleaned grid so an indoor floor plan reads
    # like an architectural drawing instead of a rainbow heatmap.
    line_segments: list[tuple[int, int, int, int]] = field(default_factory=list)
    # Degrees the cleaned grid was rotated to axis-align the dominant walls
    # (Manhattan deskew). 0.0 when alignment was off or the map was already
    # straight. The UI can surface this as "deskewed N°".
    deskew_deg: float = 0.0
    # Room footprint estimated from the cleaned grid:
    #   enclosed  bool  — interior free space is sealed by walls (a real room)
    #   length_m  float — longer side of the structure's bounding box (metres)
    #   width_m   float — shorter side (metres)
    #   area_m2   float — usable floor area (metres²)
    room: dict[str, Any] = field(default_factory=lambda: {
        "enclosed": False, "length_m": 0.0, "width_m": 0.0, "area_m2": 0.0})
    parameters: dict[str, Any] = field(default_factory=dict)

    def to_json_bytes(self) -> bytes:
        """Pack the result into a JSON blob suitable for MapRecord.map_data."""
        payload = {
            "data":           self.cleaned.flatten().astype(int).tolist(),
            "cluster_labels": self.cluster_labels.flatten().astype(int).tolist(),
            "line_segments":  [list(s) for s in self.line_segments],
            "width":          int(self.cleaned.shape[1]),
            "height":         int(self.cleaned.shape[0]),
            "n_clusters":     self.n_clusters,
            "n_noise_cells":  self.n_noise_cells,
            "n_lines":        len(self.line_segments),
            "deskew_deg":     round(float(self.deskew_deg), 2),
            "room":           self.room,
            "algorithm":      ALGORITHM_NAME,
            "parameters":     self.parameters,
        }
        return json.dumps(payload, separators=(",", ":")).encode("utf-8")


# =============================================================================
# Stage 1 — Median filter (3 × 3)
# =============================================================================
def median_3x3(arr: np.ndarray) -> np.ndarray:
    """Pure-numpy 3×3 median filter that preserves the int8 OccupancyGrid
    encoding (-1 unknown, 0 free, 100 wall). Edges are reflected so the
    output has the same shape as the input.
    """
    # Reflect-pad by 1 on each side, then build the 9-neighbourhood as a
    # (h, w, 9) stack and take the median along axis 2.
    padded = np.pad(arr, 1, mode="edge")
    h, w = arr.shape
    stack = np.empty((h, w, 9), dtype=arr.dtype)
    k = 0
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            stack[:, :, k] = padded[1 + dy : 1 + dy + h,
                                     1 + dx : 1 + dx + w]
            k += 1
    return np.median(stack, axis=2).astype(arr.dtype)


# =============================================================================
# Stage 2/3 — Morphological erosion / dilation on a binary occupied mask
# =============================================================================
def _binary_dilate(mask: np.ndarray) -> np.ndarray:
    """8-connected dilation by 1. Outside the grid is treated as False
    (zero-padded), so edge cells only grow inward."""
    h, w = mask.shape
    padded = np.pad(mask, 1, mode="constant", constant_values=False)
    out = np.zeros_like(mask)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            out |= padded[1 + dy : 1 + dy + h, 1 + dx : 1 + dx + w]
    return out


def _binary_erode(mask: np.ndarray) -> np.ndarray:
    """8-connected erosion by 1. Outside the grid is treated as False
    (zero-padded) — matches the dilation padding, which is what makes
    closing (dilate→erode) reversible on an interior shape. Slam_toolbox
    always pads its OccupancyGrid with free border cells, so walls in
    practice never sit literally on row/col 0; if they do the edge cells
    will get eroded, which is acceptable behaviour."""
    h, w = mask.shape
    padded = np.pad(mask, 1, mode="constant", constant_values=False)
    out = np.ones_like(mask)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            out &= padded[1 + dy : 1 + dy + h, 1 + dx : 1 + dx + w]
    return out


def morphological_opening(mask: np.ndarray, iterations: int = 1) -> np.ndarray:
    out = mask
    for _ in range(iterations):
        out = _binary_dilate(_binary_erode(out))
    return out


def morphological_closing(mask: np.ndarray, iterations: int = 1) -> np.ndarray:
    out = mask
    for _ in range(iterations):
        out = _binary_erode(_binary_dilate(out))
    return out


# =============================================================================
# Stage 4 — Connected-component labelling (8-connectivity, BFS)
# =============================================================================
def label_connected_components(mask: np.ndarray) -> tuple[np.ndarray, int]:
    """Label every connected blob of True cells in `mask`.

    Returns (labels, n_components) where `labels` is int32 with -1 for
    False cells and 0..n_components-1 for the i-th blob. 8-connected.

    Iterative BFS — avoids recursion-depth issues on big maps.
    """
    h, w = mask.shape
    labels = np.full((h, w), -1, dtype=np.int32)
    n = 0
    # Pre-extract True cell coordinates for fast outer loop.
    cells = np.argwhere(mask)
    for (sy, sx) in cells:
        if labels[sy, sx] != -1:
            continue
        # BFS flood-fill.
        stack = [(int(sy), int(sx))]
        while stack:
            y, x = stack.pop()
            if labels[y, x] != -1 or not mask[y, x]:
                continue
            labels[y, x] = n
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    if dy == 0 and dx == 0:
                        continue
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < h and 0 <= nx < w \
                       and mask[ny, nx] and labels[ny, nx] == -1:
                        stack.append((ny, nx))
        n += 1
    return labels, n


# =============================================================================
# Stage 5 — Hough Line Transform (wall detection)
# =============================================================================
#
# Classical algorithm — for every "on" pixel in the binary mask, vote for
# every line that could pass through it, then read off the lines that
# accumulated the most votes. Lines are parameterised in (ρ, θ) polar form,
# which avoids the slope-blows-up-on-vertical-lines problem of (m, b).
#
# Implementation is pure NumPy (no scikit-image) so the Pi-side install
# stays lean. The vectorised vote-cast handles a 200×200 cleaned grid in
# well under a second.
#
# We do *probabilistic Hough* (HoughLinesP-style) — after finding peaks in
# accumulator space, we walk along each line and emit (x0, y0, x1, y1)
# segments rather than infinite lines, with min-length + max-gap controls
# so we don't draw spurious chords across empty space.

def hough_line_segments(
    mask: np.ndarray,
    theta_steps: int = 180,
    vote_thresh: int = 18,
    min_len: int = 8,
    max_gap: int = 3,
    top_n: int = 60,
    min_fill: float = 0.0,
) -> list[tuple[int, int, int, int]]:
    """Detect straight line segments in a binary occupancy mask.

    Returns a list of (x0, y0, x1, y1) tuples in pixel/cell coordinates with
    image-style axes (origin top-left, x right, y down).

    Args:
        mask:        H×W boolean grid (True = occupied / "on" pixel)
        theta_steps: angle resolution — 180 means 1° per bin from 0..π
        vote_thresh: minimum accumulator votes per peak to consider it a line
        min_len:     minimum on-pixels along the line to keep as a segment
        max_gap:     max consecutive off-pixels before splitting a segment
        top_n:       cap on total segments emitted (strongest peaks first)
        min_fill:    minimum fraction of occupied cells between a segment's
                     endpoints; 0 disables the gate (default, for backward
                     compatibility — process_grid passes hough_min_fill).
    """
    if not isinstance(mask, np.ndarray) or mask.ndim != 2:
        raise ValueError("mask must be 2-D")
    H, W = mask.shape
    ys, xs = np.where(mask)
    if xs.size == 0:
        return []

    # ρ ranges over [-diag, +diag]. One ρ-bin per cell along the perpendicular.
    diag = int(np.ceil(np.hypot(H, W)))
    n_rho = 2 * diag + 1

    thetas = np.linspace(0.0, np.pi, theta_steps, endpoint=False, dtype=np.float32)
    cos_t = np.cos(thetas)
    sin_t = np.sin(thetas)

    # Accumulator: rows = ρ, cols = θ.
    accum = np.zeros((n_rho, theta_steps), dtype=np.int32)
    # Vectorised vote: for every "on" pixel, compute ρ for every θ, +1 in accum.
    # rhos has shape (n_pixels, n_theta). Use ROUND (not truncating cast) — in
    # float32, sin(π/2) is 0.99999964, so a horizontal line at y=10 would
    # truncate to ρ=9 and the walker would chase the wrong row, missing the
    # line entirely.
    rhos = np.round(xs[:, None] * cos_t[None, :] +
                    ys[:, None] * sin_t[None, :]).astype(np.int32) + diag
    # Bump each (rho, theta) cell by 1 per pixel that hit it.
    for ti in range(theta_steps):
        accum_col = np.bincount(rhos[:, ti], minlength=n_rho)
        accum[:, ti] += accum_col

    # Find peaks above threshold; sort by strength descending.
    peak_mask = accum >= vote_thresh
    if not peak_mask.any():
        return []
    peak_rho, peak_theta = np.where(peak_mask)
    peak_votes = accum[peak_rho, peak_theta]
    order = np.argsort(-peak_votes)
    peak_rho = peak_rho[order]
    peak_theta = peak_theta[order]

    # For each accepted peak, walk along the line and emit segments.
    # Standard probabilistic-Hough approach: parameterise the line by its
    # closest point to the origin, then step along it pixel by pixel using
    # the perpendicular direction.
    segments: list[tuple[int, int, int, int]] = []
    suppressed = np.zeros_like(accum, dtype=bool)
    NEIGH = 3  # non-max-suppression radius in accumulator space (cells)

    for rho_i, theta_i in zip(peak_rho, peak_theta):
        if suppressed[rho_i, theta_i]:
            continue
        # Suppress a small box around this peak so we don't emit ~the same line N times.
        r0, r1 = max(0, rho_i - NEIGH), min(n_rho, rho_i + NEIGH + 1)
        t0, t1 = max(0, theta_i - NEIGH), min(theta_steps, theta_i + NEIGH + 1)
        suppressed[r0:r1, t0:t1] = True

        rho = rho_i - diag
        theta = thetas[theta_i]
        ct, st = cos_t[theta_i], sin_t[theta_i]

        # Foot of the perpendicular from origin to the line.
        x0f, y0f = rho * ct, rho * st
        # Step direction along the line (perpendicular to (cos θ, sin θ)).
        dx, dy = -st, ct
        # Walk both ways from the foot until we leave the image; collect on/off bits.
        # Step in 1-pixel increments along the line.
        # Bound the walk by the diagonal so we never loop infinitely.
        on_run = []
        last_x = last_y = None
        for s in range(-diag, diag + 1):
            x = int(round(x0f + s * dx))
            y = int(round(y0f + s * dy))
            if x < 0 or x >= W or y < 0 or y >= H:
                _flush_run(on_run, segments, min_len, min_fill)
                on_run = []
                continue
            if mask[y, x]:
                on_run.append((x, y, s))
            else:
                # Tolerate small gaps before breaking the run.
                if on_run and (s - on_run[-1][2]) > max_gap:
                    _flush_run(on_run, segments, min_len, min_fill)
                    on_run = []
                elif not on_run:
                    pass
        _flush_run(on_run, segments, min_len, min_fill)
        if len(segments) >= top_n:
            break

    return segments[:top_n]


def _flush_run(run, out, min_len, min_fill=0.0):
    if len(run) < min_len:
        return
    # Fill ratio: occupied cells / cells spanned along the line. A run stitched
    # across max_gap holes can have many fewer "on" cells than its span — that's
    # a chord through open space, not a wall. Reject it.
    if min_fill > 0.0:
        span = run[-1][2] - run[0][2] + 1   # cells along the line (s is the step index)
        if span > 0 and (len(run) / span) < min_fill:
            return
    x0, y0, _ = run[0]
    x1, y1, _ = run[-1]
    out.append((int(x0), int(y0), int(x1), int(y1)))


# =============================================================================
# Stage 6 — Manhattan-world regularisation (orientation cleanup)
# =============================================================================
#
# A handheld scan ends up tilted at some arbitrary angle, and yaw drift makes
# the same wall land at slightly different angles on different passes. Real
# indoor spaces are overwhelmingly rectilinear, so we estimate the single
# dominant wall direction and rotate the whole grid to put it on the axes.
# Every room is then aligned to the same reference and the plan reads cleanly.

def estimate_deskew_angle(
    mask: np.ndarray,
    theta_steps: int = 180,
    min_votes: int = 8,
    min_concentration: float = 0.2,
) -> float:
    """Return the rotation (radians, in (-π/4, π/4]) that axis-aligns the
    dominant wall direction in a binary occupancy `mask`.

    Walls are detected the same way as the Hough transform — peaks in the
    (ρ, θ) accumulator — but here we only need the *angle* of the strongest
    structure. Each θ-bin's strongest line votes for its orientation; we fold
    orientation modulo 90° (so the two perpendicular wall families of a
    rectangular room reinforce a single estimate) by mapping the angle into
    the doubled domain and taking the vote-weighted circular mean.

    Returns 0.0 (no rotation) when there isn't enough linear structure
    (`min_votes`) or when the walls don't share a dominant orientation — the
    vote-vector concentration ``R = |Σ w·e^{i4θ}| / Σ w`` below
    `min_concentration`. A clean room scores R ≈ 0.33 even when tilted; a
    drift-smeared multi-room scan scores R ≈ 0.1 and is left alone so the
    deskew can never make a messy map worse.
    """
    if not isinstance(mask, np.ndarray) or mask.ndim != 2:
        raise ValueError("mask must be 2-D")
    ys, xs = np.where(mask)
    if xs.size == 0:
        return 0.0

    H, W = mask.shape
    diag = int(np.ceil(np.hypot(H, W)))
    n_rho = 2 * diag + 1
    thetas = np.linspace(0.0, np.pi, theta_steps, endpoint=False, dtype=np.float32)
    cos_t, sin_t = np.cos(thetas), np.sin(thetas)
    rhos = np.round(xs[:, None] * cos_t[None, :] +
                    ys[:, None] * sin_t[None, :]).astype(np.int32) + diag

    # Per-θ strength = the single strongest line at that orientation.
    strength = np.zeros(theta_steps, dtype=np.int64)
    for ti in range(theta_steps):
        strength[ti] = np.bincount(rhos[:, ti], minlength=n_rho).max()

    total = int(strength.sum())
    if int(strength.max()) < min_votes or total == 0:
        return 0.0  # nothing wall-like enough to align to

    # Vote-weighted circular mean of θ folded modulo 90° → multiply the angle
    # by 4 so a 90° period maps onto a full 2π circle, average as unit
    # vectors, divide back. Robust to the wrap at 0/90°.
    z = np.sum(strength * np.exp(1j * 4.0 * thetas.astype(np.float64)))
    concentration = abs(z) / total
    if concentration < min_concentration:
        return 0.0  # walls point every which way — don't fabricate alignment
    phi = math.atan2(z.imag, z.real) / 4.0   # dominant orientation in (-π/4, π/4]
    # Rotate the grid by -phi to bring that orientation onto an axis.
    return -phi


def estimate_tilt_from_segments(
    segments: list[tuple[int, int, int, int]],
) -> tuple[float, float, float]:
    """Estimate the map tilt from detected wall segments.

    This is the deskew estimator ``process_grid`` actually uses. The raw
    accumulator variant (``estimate_deskew_angle``) is diluted by wall smear
    on real handheld maps — every θ-bin scores similar votes across a thick
    blurry wall, so the angular concentration lands under any usable gate
    and the deskew never fires. Merged Hough segments are already the
    de-smeared wall hypotheses, so their length-weighted mean orientation is
    a far sharper signal (measured R = 0.75–0.99 on field maps vs 0.16–0.21
    for the accumulator on the same data).

    Args:
        segments: (x0, y0, x1, y1) wall segments, ideally post-merge.

    Returns:
        (angle_rad, concentration, total_len) where ``angle_rad`` is the
        rotation to APPLY to axis-align the dominant wall family (same sign
        convention as ``estimate_deskew_angle``), ``concentration`` is the
        length-weighted resultant R ∈ [0, 1] of the segment angles folded
        modulo 90°, and ``total_len`` is the summed segment length in cells.
        All zeros when there are no usable segments.
    """
    zx = zy = total = 0.0
    for x0, y0, x1, y1 in segments:
        length = math.hypot(x1 - x0, y1 - y0)
        if length < 1e-9:
            continue
        phi = math.atan2(y1 - y0, x1 - x0)
        # Fold mod 90° (×4 maps a 90° period onto the full circle) so the
        # two perpendicular wall families reinforce one estimate.
        zx += length * math.cos(4.0 * phi)
        zy += length * math.sin(4.0 * phi)
        total += length
    if total <= 0.0:
        return 0.0, 0.0, 0.0
    concentration = math.hypot(zx, zy) / total
    phi = math.atan2(zy, zx) / 4.0           # dominant orientation (-π/4, π/4]
    return -phi, concentration, total


def rotate_grid_nn(grid: np.ndarray, angle_rad: float, fill: int) -> np.ndarray:
    """Nearest-neighbour rotation of `grid` about its centre by `angle_rad`,
    expanding the canvas so nothing is clipped. Out-of-source pixels get
    `fill`. Pure-numpy inverse warp (no scipy) — exact for the int8/int32
    label grids we rotate, with no interpolation across class boundaries."""
    h, w = grid.shape
    cos_a, sin_a = math.cos(angle_rad), math.sin(angle_rad)
    new_w = int(math.ceil(abs(w * cos_a) + abs(h * sin_a)))
    new_h = int(math.ceil(abs(w * sin_a) + abs(h * cos_a)))
    cx, cy = (w - 1) / 2.0, (h - 1) / 2.0
    ncx, ncy = (new_w - 1) / 2.0, (new_h - 1) / 2.0

    yy, xx = np.indices((new_h, new_w))
    dx = xx - ncx
    dy = yy - ncy
    # Inverse map output→input (rotate by -angle), then nearest-neighbour.
    src_x = np.round(cos_a * dx + sin_a * dy + cx).astype(np.int32)
    src_y = np.round(-sin_a * dx + cos_a * dy + cy).astype(np.int32)
    valid = (src_x >= 0) & (src_x < w) & (src_y >= 0) & (src_y < h)
    out = np.full((new_h, new_w), fill, dtype=grid.dtype)
    out[valid] = grid[src_y[valid], src_x[valid]]
    return out


def snap_segments(
    segments: list[tuple[int, int, int, int]],
    snap_deg: float,
) -> list[tuple[int, int, int, int]]:
    """Snap each segment within `snap_deg` of horizontal/vertical to exactly
    horizontal/vertical (sharing the rounded mean of the off-axis coordinate).
    Segments more than `snap_deg` off an axis (genuine diagonals) pass
    through untouched."""
    if snap_deg <= 0:
        return segments
    out: list[tuple[int, int, int, int]] = []
    for x0, y0, x1, y1 in segments:
        ang = math.degrees(math.atan2(y1 - y0, x1 - x0)) % 180.0
        if min(ang, abs(ang - 180.0)) <= snap_deg:          # near-horizontal
            y = int(round((y0 + y1) / 2.0))
            out.append((int(x0), y, int(x1), y))
        elif abs(ang - 90.0) <= snap_deg:                    # near-vertical
            x = int(round((x0 + x1) / 2.0))
            out.append((x, int(y0), x, int(y1)))
        else:
            out.append((int(x0), int(y0), int(x1), int(y1)))
    return out


def orthogonal_snap(
    segments: list[tuple[int, int, int, int]],
    snap_deg: float,
) -> list[tuple[int, int, int, int]]:
    """Regularise wall orientations onto one orthogonal grid so near-right-angle
    corners come out exactly 90°.

    The dominant direction θ0 is the length-weighted circular mean of the
    segment orientations folded modulo 90° (multiply the angle by 4 so the two
    perpendicular wall families of a rectangular room reinforce one estimate,
    then divide back — θ0 lands in (-45°, 45°]). Each segment whose orientation
    is within `snap_deg` of θ0 + k·90° is rotated about its own midpoint to that
    exact angle, preserving its length and centre: parallel walls stay parallel,
    perpendicular walls become exactly square. Segments more than `snap_deg` off
    every grid line (genuine odd-angle walls) pass through untouched.

    Unlike `snap_segments`, which only snaps to the absolute image axes, this
    squares corners at *any* global tilt — useful when the deskew left the map
    rotated (or was disabled) but the room is still rectilinear.
    """
    if snap_deg <= 0 or not segments:
        return list(segments)

    # Dominant direction θ0, folded mod 90°.
    zx = zy = 0.0
    for x0, y0, x1, y1 in segments:
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy)
        if L < 1e-9:
            continue
        phi = math.atan2(dy, dx)
        zx += L * math.cos(4.0 * phi)
        zy += L * math.sin(4.0 * phi)
    if zx == 0.0 and zy == 0.0:
        return list(segments)
    theta0 = math.atan2(zy, zx) / 4.0
    tol = math.radians(snap_deg)
    half_pi = math.pi / 2.0

    out: list[tuple[int, int, int, int]] = []
    for x0, y0, x1, y1 in segments:
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy)
        if L < 1e-9:
            out.append((int(x0), int(y0), int(x1), int(y1)))
            continue
        phi = math.atan2(dy, dx)
        # Nearest grid line θ0 + k·90°.
        k = round((phi - theta0) / half_pi)
        target = theta0 + k * half_pi
        d = abs(math.atan2(math.sin(phi - target), math.cos(phi - target)))
        if d <= tol:
            mx, my = (x0 + x1) / 2.0, (y0 + y1) / 2.0
            hx, hy = (L / 2.0) * math.cos(target), (L / 2.0) * math.sin(target)
            out.append((int(round(mx - hx)), int(round(my - hy)),
                        int(round(mx + hx)), int(round(my + hy))))
        else:
            out.append((int(x0), int(y0), int(x1), int(y1)))
    return out


# =============================================================================
# Stage 7 — Collinear segment merge (wall de-duplication)
# =============================================================================
#
# The Hough accumulator does not report "one peak per wall". A single physical
# wall — especially a thick or slightly noisy one — lights up a *cluster* of
# neighbouring (ρ, θ) cells, and the probabilistic walker fragments each into
# several segments. The net effect is the "60 walls" artefact: a clean 4-wall
# room renders as dozens of overlapping cyan strokes.
#
# The fix is to recognise that those segments are *hypotheses about the same
# wall* and collapse each group to one representative. Conceptually this is a
# clustering problem in line-parameter space (orientation, perpendicular
# offset).
#
# ── "Can a fuzzy algorithm be used here?" ────────────────────────────────────
# This merge IS the place where a fuzzy method would live. Fuzzy c-means (or
# fuzzy-DBSCAN) would assign each Hough segment a soft membership to a set of
# wall prototypes and fuse the high-membership ones. We deliberately use the
# *crisp* equivalent instead — single-link grouping by an (angle, offset)
# tolerance — because:
#   • fuzzy c-means needs the number of walls k up front (we don't know it),
#   • it is non-deterministic (random init) — bad for a reproducible map,
#   • and on this 1-D-ish parameter space the soft memberships collapse to the
#     same partition the tolerance test gives, for no extra benefit.
# The one spot where genuine fuzzy logic would add value is the orthogonal
# *snap* (snap_segments): replacing its hard ``snap_deg`` cut-off with a smooth
# "how horizontal/vertical is this wall" membership would remove the cliff at
# exactly snap_deg. That is a polish, not the cause of the 60-walls problem.

def _seg_geometry(seg: tuple[int, int, int, int]):
    """Return (unit-normal nx, ny, point x0, y0, orientation θ∈[0,π), length)."""
    x0, y0, x1, y1 = seg
    dx, dy = x1 - x0, y1 - y0
    length = math.hypot(dx, dy)
    if length < 1e-9:
        return 0.0, 1.0, float(x0), float(y0), 0.0, 0.0
    nx, ny = -dy / length, dx / length            # unit normal to the segment
    theta = math.atan2(dy, dx) % math.pi          # direction folded into [0,π)
    return nx, ny, float(x0), float(y0), theta, length


def _angle_dist(a: float, b: float) -> float:
    """Smallest angle between two orientations with period π (radians)."""
    d = abs(a - b) % math.pi
    return min(d, math.pi - d)


def merge_collinear_segments(
    segments: list[tuple[int, int, int, int]],
    angle_deg: float = 7.0,
    offset: float = 3.0,
    gap: float = 8.0,
) -> list[tuple[int, int, int, int]]:
    """Collapse the Hough "bundle" of near-duplicate segments into one wall each.

    Two segments belong to the same wall when their orientations differ by at
    most ``angle_deg`` AND each segment's midpoint lies within ``offset`` cells
    of the other's supporting line. Groups are formed by single-link
    agglomeration (transitive). Within a group the endpoints are projected onto
    the length-weighted consensus line and intervals that overlap or sit within
    ``gap`` cells of each other are unioned; one segment is emitted per merged
    interval. Returns segments sorted longest-first.
    """
    if len(segments) <= 1:
        return list(segments)

    geom = [_seg_geometry(s) for s in segments]
    n = len(segments)
    ang_tol = math.radians(angle_deg)

    def same_wall(i: int, j: int) -> bool:
        nxi, nyi, xi, yi, ti, _ = geom[i]
        nxj, nyj, xj, yj, tj, _ = geom[j]
        if _angle_dist(ti, tj) > ang_tol:
            return False
        mxi, myi = (segments[i][0] + segments[i][2]) / 2.0, (segments[i][1] + segments[i][3]) / 2.0
        mxj, myj = (segments[j][0] + segments[j][2]) / 2.0, (segments[j][1] + segments[j][3]) / 2.0
        # Perpendicular distance of each midpoint to the other's line (both ways
        # so a short fragment near a long wall still matches).
        di = abs(nxj * (mxi - xj) + nyj * (myi - yj))
        dj = abs(nxi * (mxj - xi) + nyi * (myj - yi))
        return min(di, dj) <= offset

    # Single-link union-find over the (small) segment set.
    parent = list(range(n))
    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for i in range(n):
        for j in range(i + 1, n):
            if same_wall(i, j):
                parent[find(i)] = find(j)

    groups: dict[int, list[int]] = {}
    for i in range(n):
        groups.setdefault(find(i), []).append(i)

    merged: list[tuple[int, int, int, int]] = []
    for members in groups.values():
        # Consensus direction = length-weighted circular mean of 2θ (period π).
        zx = zy = 0.0
        for m in members:
            _, _, _, _, t, L = geom[m]
            w = L + 1.0
            zx += w * math.cos(2 * t)
            zy += w * math.sin(2 * t)
        theta = math.atan2(zy, zx) / 2.0
        ux, uy = math.cos(theta), math.sin(theta)
        # Length-weighted centroid of all endpoints = anchor on the consensus line.
        sw = ax = ay = 0.0
        for m in members:
            x0, y0, x1, y1 = segments[m]
            _, _, _, _, _, L = geom[m]
            w = L + 1.0
            ax += w * (x0 + x1) / 2.0
            ay += w * (y0 + y1) / 2.0
            sw += w
        ax, ay = ax / sw, ay / sw
        # Project endpoints onto the line direction → 1-D intervals.
        intervals: list[tuple[float, float]] = []
        for m in members:
            x0, y0, x1, y1 = segments[m]
            t0 = (x0 - ax) * ux + (y0 - ay) * uy
            t1 = (x1 - ax) * ux + (y1 - ay) * uy
            intervals.append((min(t0, t1), max(t0, t1)))
        # Union intervals that overlap or are within `gap`.
        intervals.sort()
        cur_lo, cur_hi = intervals[0]
        unioned: list[tuple[float, float]] = []
        for lo, hi in intervals[1:]:
            if lo <= cur_hi + gap:
                cur_hi = max(cur_hi, hi)
            else:
                unioned.append((cur_lo, cur_hi))
                cur_lo, cur_hi = lo, hi
        unioned.append((cur_lo, cur_hi))
        for lo, hi in unioned:
            merged.append((
                int(round(ax + lo * ux)), int(round(ay + lo * uy)),
                int(round(ax + hi * ux)), int(round(ay + hi * uy)),
            ))

    merged.sort(key=lambda s: math.hypot(s[2] - s[0], s[3] - s[1]), reverse=True)
    return merged


# =============================================================================
# Stage 8 — Bake straightened walls + room metrics
# =============================================================================
#
# After detection the walls live as a handful of clean (x0,y0,x1,y1) segments.
# Rasterising them back into the grid as ordinary wall cells — and clearing the
# ragged scan boundary they replace — turns the occupancy map itself into a
# tidy floor plan, so the UI needs no neon overlay: walls just render uniformly.

def _bresenham(x0: int, y0: int, x1: int, y1: int):
    """Integer line cells from (x0,y0) to (x1,y1), inclusive (Bresenham)."""
    pts = []
    dx, dy = abs(x1 - x0), -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        pts.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x += sx
        if e2 <= dx:
            err += dx
            y += sy
    return pts


def rasterize_segments(
    segments: list[tuple[int, int, int, int]],
    shape: tuple[int, int],
    thickness: int = 1,
) -> np.ndarray:
    """Draw `segments` as occupied (True) cells on an H×W boolean grid,
    dilated to `thickness` cells wide."""
    H, W = shape
    mask = np.zeros((H, W), dtype=bool)
    for x0, y0, x1, y1 in segments:
        for x, y in _bresenham(int(x0), int(y0), int(x1), int(y1)):
            if 0 <= x < W and 0 <= y < H:
                mask[y, x] = True
    for _ in range(max(0, int(thickness) - 1)):
        mask = _binary_dilate(mask)
    return mask


def room_metrics(
    cleaned: np.ndarray,
    occupied_threshold: int,
    resolution: float,
    seal: int = 2,
    bbox_segments: list[tuple[int, int, int, int]] | None = None,
) -> dict[str, Any]:
    """Estimate the room footprint from a cleaned occupancy grid.

    Returns ``{enclosed, length_m, width_m, area_m2}``:

      * ``length_m`` / ``width_m`` — the longer / shorter side of the bounding
        box of the wall structure, in metres (``cells × resolution``). Because
        this runs *after* the Manhattan deskew, the bounding box is axis-aligned
        with the room, so these are the real room dimensions.
      * ``enclosed`` — True when the interior free space is sealed off from the
        grid border by walls (i.e. the scan is *inside a room*, not an open or
        partial sweep). Computed by flood-filling the "outside": label the
        non-wall cells, and any free cell whose component does **not** touch the
        grid border is interior. Walls are first dilated by ``seal`` cells so a
        normal doorway gap doesn't leak the interior to the outside and make a
        real room read as "open".
      * ``area_m2`` — usable floor area: interior free-cell count × cell² when
        enclosed, else the total free area as a best-effort fallback.
    """
    res = float(resolution) if resolution and float(resolution) > 0 else 0.05
    out: dict[str, Any] = {
        "enclosed": False, "length_m": 0.0, "width_m": 0.0, "area_m2": 0.0}

    occ = cleaned >= occupied_threshold
    if not occ.any():
        return out

    free = cleaned == 0
    # Bounding box — from wall centre-lines if given (so thick baked walls don't
    # inflate the dimensions), else from the occupied cells.
    if bbox_segments:
        sxs = [c for s in bbox_segments for c in (s[0], s[2])]
        sys = [c for s in bbox_segments for c in (s[1], s[3])]
        side_a = (max(sys) - min(sys) + 1) * res
        side_b = (max(sxs) - min(sxs) + 1) * res
    else:
        ys, xs = np.where(occ)
        side_a = int(ys.max() - ys.min() + 1) * res
        side_b = int(xs.max() - xs.min() + 1) * res
    out["length_m"] = round(max(side_a, side_b), 2)
    out["width_m"] = round(min(side_a, side_b), 2)

    # Seal small openings, then find free space that can't reach the border.
    occ_sealed = occ.copy()
    for _ in range(max(0, int(seal))):
        occ_sealed = _binary_dilate(occ_sealed)
    labels, n = label_connected_components(~occ_sealed)
    interior_free = 0
    total_free = int(free.sum())
    if n > 0:
        border = np.concatenate(
            [labels[0, :], labels[-1, :], labels[:, 0], labels[:, -1]])
        border_labels = [int(v) for v in np.unique(border) if v >= 0]
        interior = (labels >= 0) & ~np.isin(labels, border_labels)
        interior_free = int((interior & free).sum())

    out["enclosed"] = bool(
        total_free > 0 and interior_free >= 0.5 * total_free and interior_free >= 20)
    floor_cells = interior_free if out["enclosed"] else total_free
    out["area_m2"] = round(floor_cells * res * res, 2)
    return out


# =============================================================================
# Top-level pipeline
# =============================================================================
def process_grid(
    cell_values: list[int] | np.ndarray,
    width: int,
    height: int,
    params: dict[str, Any] | None = None,
) -> ProcessResult:
    """Run the full Tier 2 pipeline on a flattened OccupancyGrid.

    Args:
        cell_values: row-major list of int8 cell values (-1, 0..100).
        width, height: grid dimensions.
        params: optional override of DEFAULT_PARAMS.

    Returns:
        ProcessResult with the cleaned grid + cluster labels + counts.
    """
    p = {**DEFAULT_PARAMS, **(params or {})}
    arr = np.asarray(cell_values, dtype=np.int8).reshape((height, width))

    # Stage 1: median.
    cleaned = median_3x3(arr) if p["median_size"] == 3 else arr

    # Stages 2 / 3: morphology on the occupied mask. The free/unknown
    # parts of the grid are passed through unchanged — we only operate
    # on cells that the OccupancyGrid considers "wall-like".
    occupied = cleaned >= p["occupied_threshold"]
    occupied = morphological_opening(occupied, p["opening_iterations"])
    occupied = morphological_closing(occupied, p["closing_iterations"])

    # Rebuild the cleaned grid: keep original free / unknown classification
    # for non-occupied cells, force 100 wherever the morphology says
    # "occupied", and 0 wherever it says "not occupied" but the original
    # was occupied (an artefact we filtered out).
    cleaned_out = cleaned.copy()
    was_occupied = arr >= p["occupied_threshold"]
    # newly-cleared cells (previously occupied, now not) → treat as free
    cleared = was_occupied & ~occupied
    cleaned_out[cleared] = 0
    # newly-filled cells (previously not occupied, now are) — rare with
    # closing-only ops, but if it happens mark as wall.
    cleaned_out[~was_occupied & occupied] = 100

    # Stage 4: connected components on the post-morph occupied mask.
    raw_labels, raw_n = label_connected_components(occupied)

    # Apply the min-cluster-size filter: any cluster smaller than the
    # threshold is reclassified as noise (label = -1, and the cell is
    # reset to free in the cleaned output).
    min_size = max(1, int(p["min_cluster_size"]))
    cluster_labels = np.full_like(raw_labels, -1)
    if raw_n > 0:
        sizes = np.bincount(raw_labels[raw_labels >= 0].ravel(),
                            minlength=raw_n)
        next_label = 0
        for old_label, sz in enumerate(sizes):
            if sz < min_size:
                continue
            cluster_labels[raw_labels == old_label] = next_label
            next_label += 1
        # Noise cleanup: cells that lost their cluster get marked free.
        cleared_by_size = (raw_labels >= 0) & (cluster_labels < 0)
        cleaned_out[cleared_by_size] = 0
        n_clusters = next_label
    else:
        n_clusters = 0

    n_noise_cells = int(((raw_labels >= 0) & (cluster_labels < 0)).sum())

    # Stage 4.5: free-space opening. Handheld scans carve thin "free" rays
    # through windows / door gaps (a radial fan of streaks around the room —
    # 10–17 % of all free cells on real field maps). Opening the free mask
    # dissolves anything thinner than 3 cells; the removed cells revert to
    # unknown (-1) because the scan has no reliable information there.
    if int(p["free_opening_iterations"]) > 0:
        free_mask = cleaned_out == 0
        opened_free = morphological_opening(
            free_mask, int(p["free_opening_iterations"]))
        cleaned_out[free_mask & ~opened_free] = -1

    def _detect(mask: np.ndarray) -> list[tuple[int, int, int, int]]:
        return hough_line_segments(
            mask,
            theta_steps=int(p["hough_theta_steps"]),
            vote_thresh=int(p["hough_vote_thresh"]),
            min_len=int(p["hough_min_len"]),
            max_gap=int(p["hough_max_gap"]),
            top_n=int(p["hough_top_n"]),
            min_fill=float(p["hough_min_fill"]),
        )

    # Stage 5: Hough Line Transform on the cleaned occupied mask. The web UI
    # overlays these segments on top of the cleaned grid so an indoor floor
    # plan reads like an architectural drawing.
    line_segments = _detect(cleaned_out >= p["occupied_threshold"])

    # Stage 6: Manhattan deskew. The tilt comes from the MERGED wall segments
    # (see estimate_tilt_from_segments — the raw accumulator estimate is
    # diluted below any usable concentration gate by wall smear on real
    # handheld maps, which left the deskew permanently dormant). The merge
    # here is internal to the estimate; the emitted segment list is merged
    # later in the normal chain.
    deskew_deg = 0.0
    if int(p["manhattan_align"]) and line_segments:
        angle, concentration, wall_cells = estimate_tilt_from_segments(
            merge_collinear_segments(
                line_segments,
                angle_deg=float(p["merge_angle_deg"]),
                offset=float(p["merge_offset"]),
                gap=float(p["merge_gap"]),
            ))
        if (wall_cells >= float(p["manhattan_min_wall_cells"])
                and concentration >= float(p["manhattan_min_concentration"])
                and abs(math.degrees(angle)) >= float(p["manhattan_min_deg"])):
            cleaned_out = rotate_grid_nn(cleaned_out, angle, fill=-1)
            cluster_labels = rotate_grid_nn(cluster_labels, angle, fill=-1)
            deskew_deg = math.degrees(angle)
            # Re-detect on the deskewed mask so the emitted segments live in
            # the deskewed frame (rotating the segments analytically would
            # drift off the NN-rotated grid by up to a cell).
            line_segments = _detect(cleaned_out >= p["occupied_threshold"])

    # After deskew the walls are near-axis-aligned — snap near-orthogonal
    # segments to exactly H/V so the overlay is crisp.
    if int(p["manhattan_align"]):
        line_segments = snap_segments(line_segments, float(p["manhattan_snap_deg"]))
    # Square up corners: force near-right-angle walls to be exactly parallel /
    # perpendicular to the dominant wall direction (works at any global tilt).
    if int(p["ortho_snap"]):
        line_segments = orthogonal_snap(line_segments, float(p["ortho_snap_deg"]))

    # Stage 7: collapse the Hough "bundle" of duplicate/fragmented segments into
    # one wall each (snap first so H/V fragments share an exact orientation and
    # merge cleanly), then keep only the longest walls. This is what turns "60
    # walls" back into the handful a real room actually has.
    if int(p["merge_segments"]):
        line_segments = merge_collinear_segments(
            line_segments,
            angle_deg=float(p["merge_angle_deg"]),
            offset=float(p["merge_offset"]),
            gap=float(p["merge_gap"]),
        )
    max_walls = int(p["max_walls"])
    if max_walls > 0 and len(line_segments) > max_walls:
        line_segments = sorted(
            line_segments,
            key=lambda s: math.hypot(s[2] - s[0], s[3] - s[1]),
            reverse=True,
        )[:max_walls]

    # Stage 8: bake the detected straight walls back into the grid as uniform
    # wall cells, absorbing the ragged scan boundary they replace, so the map
    # itself reads like a floor plan (no overlay). Only occupied cells within
    # `wall_absorb` dilations of a baked wall are cleared — they are the
    # ragged/smeared evidence the straight wall explains. Occupied structure
    # farther from every detected wall (furniture, walls the detector missed)
    # is kept: clearing it to free — the old behaviour — fabricated open
    # floor out of real obstacles whenever detection was incomplete, which on
    # smeared handheld maps was every time.
    thr = int(p["occupied_threshold"])
    if int(p["bake_walls"]) and line_segments:
        thick_mask = rasterize_segments(
            line_segments, cleaned_out.shape, thickness=int(p["wall_thickness"]))
        absorb_zone = thick_mask
        for _ in range(max(0, int(p["wall_absorb"]))):
            absorb_zone = _binary_dilate(absorb_zone)
        absorbed = (cleaned_out >= thr) & absorb_zone & ~thick_mask
        cleaned_out[absorbed] = 0             # ragged boundary → free
        cleaned_out[thick_mask] = 100         # draw uniform thick walls
        # n_clusters / cluster_labels keep describing the *detected obstacles*
        # (the clustering stage) — they are not re-derived from the baked walls.

    # Room footprint + "are we inside a room?" detection. Measured on the baked
    # walls: the thick, continuous lines seal corners far better than the ragged
    # input boundary, so the enclosure flood-fill is reliable. The bounding box
    # is taken from the wall centre-lines (segment endpoints) so wall thickness
    # doesn't inflate the reported length/width.
    room = room_metrics(
        cleaned_out,
        occupied_threshold=thr,
        resolution=float(p["resolution"]),
        seal=int(p["room_seal"]),
        bbox_segments=line_segments if int(p["bake_walls"]) else None,
    )

    return ProcessResult(
        cleaned=cleaned_out,
        cluster_labels=cluster_labels,
        n_clusters=n_clusters,
        n_noise_cells=n_noise_cells,
        line_segments=line_segments,
        deskew_deg=deskew_deg,
        room=room,
        parameters=p,
    )
