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
}


@dataclass
class ProcessResult:
    cleaned: np.ndarray            # int8 (h, w) — same encoding as input
    cluster_labels: np.ndarray     # int32 (h, w) — -1 = not-occupied / noise
    n_clusters: int
    n_noise_cells: int
    parameters: dict[str, Any] = field(default_factory=dict)

    def to_json_bytes(self) -> bytes:
        """Pack the result into a JSON blob suitable for MapRecord.map_data."""
        payload = {
            "data":           self.cleaned.flatten().astype(int).tolist(),
            "cluster_labels": self.cluster_labels.flatten().astype(int).tolist(),
            "width":          int(self.cleaned.shape[1]),
            "height":         int(self.cleaned.shape[0]),
            "n_clusters":     self.n_clusters,
            "n_noise_cells":  self.n_noise_cells,
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

    return ProcessResult(
        cleaned=cleaned_out,
        cluster_labels=cluster_labels,
        n_clusters=n_clusters,
        n_noise_cells=n_noise_cells,
        parameters=p,
    )
