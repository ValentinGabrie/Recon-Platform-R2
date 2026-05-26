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

The output cluster grid uses ``-1`` for "not occupied" (free or unknown
cells in the cleaned grid) and ``0, 1, 2, …`` for each distinct cluster.
"""

from __future__ import annotations

import json
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
    # --- Hough Line Transform (wall detection) -------------------------------
    # `hough_theta_steps`  number of angle bins from 0..π (1° resolution = 180)
    # `hough_vote_thresh`  minimum accumulator votes for a peak to count as a line
    # `hough_min_len`      minimum on-pixels along the line to emit a segment (cells)
    # `hough_max_gap`      max gap between on-pixels before splitting a segment (cells)
    # `hough_top_n`        emit at most this many strongest line segments
    "hough_theta_steps":   180,
    "hough_vote_thresh":   18,
    "hough_min_len":       8,
    "hough_max_gap":       3,
    "hough_top_n":         60,
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
                _flush_run(on_run, segments, min_len)
                on_run = []
                continue
            if mask[y, x]:
                on_run.append((x, y, s))
            else:
                # Tolerate small gaps before breaking the run.
                if on_run and (s - on_run[-1][2]) > max_gap:
                    _flush_run(on_run, segments, min_len)
                    on_run = []
                elif not on_run:
                    pass
        _flush_run(on_run, segments, min_len)
        if len(segments) >= top_n:
            break

    return segments[:top_n]


def _flush_run(run, out, min_len):
    if len(run) < min_len:
        return
    x0, y0, _ = run[0]
    x1, y1, _ = run[-1]
    out.append((int(x0), int(y0), int(x1), int(y1)))


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

    # Stage 5: Hough Line Transform on the (post-cleanup) occupied mask. The
    # web UI overlays these segments on top of the cleaned grid so an indoor
    # floor plan reads like an architectural drawing.
    final_occupied = cleaned_out >= p["occupied_threshold"]
    line_segments = hough_line_segments(
        final_occupied,
        theta_steps=int(p["hough_theta_steps"]),
        vote_thresh=int(p["hough_vote_thresh"]),
        min_len=int(p["hough_min_len"]),
        max_gap=int(p["hough_max_gap"]),
        top_n=int(p["hough_top_n"]),
    )

    return ProcessResult(
        cleaned=cleaned_out,
        cluster_labels=cluster_labels,
        n_clusters=n_clusters,
        n_noise_cells=n_noise_cells,
        line_segments=line_segments,
        parameters=p,
    )
