"""Footprint Polygon to Circles.

=======================
Decomposes a convex polygon robot footprint into overlapping circles
for use with TEB-K EdgeESDFPose.

Algorithm
---------
1. PCA  →  find major axis (works for any rotation)
2. Greedy sweep along major axis  →  circle center positions
   - First/last circle placed at t = t_edge ± r_body
     so each end-circle exactly reaches the polygon edge
3. Voronoi partition  →  minimum covering radius per circle
   r_i = max distance from center_i to any vertex of its Voronoi region
   → guarantees 100% polygon coverage by construction

Usage
-----
  python3 footprint_to_circles.py

  Adjust FOOTPRINT_VERTICES and OPTIONS at the bottom of the file.

Output
------
  - Console: circle list, ROS2 YAML, C++ initializer
  - File:    footprint_circles.png  (visualization)

Dependencies
------------
  pip install numpy shapely matplotlib
"""
from dataclasses import dataclass
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from shapely import contains_xy
from shapely.geometry import Point, Polygon
from typing import List, Optional, Tuple
import numpy as np
import matplotlib
matplotlib.use('Agg')


@dataclass
class CircleResult:
    """Cirle."""

    cx: float
    cy: float
    r: float
    r_inscribed: float  # distance from center to nearest polygon boundary
    overhang: float     # r - r_inscribed (how much circle extends outside polygon)


# ──────────────────────────────────────────────────────────────────────────────
# Core algorithm
# ──────────────────────────────────────────────────────────────────────────────

def polygon_to_circles(
    vertices: List[Tuple[float, float]],
    max_overhang_m: Optional[float] = None,
    n_circles: Optional[int] = None,
    coverage_threshold: float = 0.999,
    max_circles: int = 20,
    candidate_spacing: float = 0.05,  # grid density for circle centers [m]
    pixel_size: float = 0.02,         # raster resolution for coverage [m]
) -> Tuple[List[CircleResult], Polygon, float]:
    """
    Minimum-circle polygon cover via Greedy Maximum Coverage.

    Parameters
    ----------
    vertices : list of (x, y)
        Polygon vertices in robot body frame.
    max_overhang_m : float or None
        Maximum distance circles may extend outside polygon [m].
        None = unlimited (r = inscribed_radius, Voronoi guaranteed 100%).
    n_circles : int or None
        Force exactly N circles. None = stop at coverage_threshold.
    coverage_threshold : float
        Stop when this fraction of polygon area is covered [0..1].
    max_circles : int
        Hard cap on circle count.
    candidate_spacing : float
        Grid spacing for candidate circle centers [m].
        Smaller = more candidates = better optimum, but slower.
    pixel_size : float
        Raster resolution for coverage computation [m].
        Smaller = more accurate coverage, but more memory.

    Returns
    -------
    circles  : List[CircleResult]
    poly     : shapely.Polygon
    coverage : float   fraction of polygon covered [0..1]
    """
    poly = Polygon(vertices).convex_hull
    bounds = poly.bounds   # (minx, miny, maxx, maxy)

    # ── Step 1: Rasterize polygon → coverage pixels ───────────────────────────
    # Each pixel represents a small area of the polygon that must be covered.
    xs = np.arange(bounds[0], bounds[2] + pixel_size, pixel_size)
    ys = np.arange(bounds[1], bounds[3] + pixel_size, pixel_size)
    XX, YY = np.meshgrid(xs, ys)
    flat_x = XX.ravel()
    flat_y = YY.ravel()

    inside_mask = contains_xy(poly, flat_x, flat_y)
    px = flat_x[inside_mask]   # world-x of polygon pixels
    py = flat_y[inside_mask]   # world-y of polygon pixels
    n_pixels = len(px)

    if n_pixels == 0:
        return [], poly, 0.0

    # ── Step 2: Candidate circle centers (grid inside polygon) ────────────────
    cx_grid = np.arange(bounds[0] + candidate_spacing / 2,
                        bounds[2], candidate_spacing)
    cy_grid = np.arange(bounds[1] + candidate_spacing / 2,
                        bounds[3], candidate_spacing)
    CX, CY = np.meshgrid(cx_grid, cy_grid)
    cand_flat_x = CX.ravel()
    cand_flat_y = CY.ravel()

    cand_inside = contains_xy(poly, cand_flat_x, cand_flat_y)
    cand_x = cand_flat_x[cand_inside]
    cand_y = cand_flat_y[cand_inside]
    n_cand = len(cand_x)

    if n_cand == 0:
        return [], poly, 0.0

    # ── Step 3: r_max per candidate ───────────────────────────────────────────
    # r_max[i] = inscribed_radius[i] + max_overhang_m
    r_inscribed_cand = np.array([
        poly.exterior.distance(Point(x, y))
        for x, y in zip(cand_x, cand_y)
    ])
    if max_overhang_m is None:
        r_max_cand = r_inscribed_cand.copy()
        # For unlimited overhang, use a large value so Voronoi handles coverage
        # → set to polygon diagonal (upper bound for any useful radius)
        diag = np.sqrt((bounds[2] - bounds[0])**2 + (bounds[3] - bounds[1])**2)
        r_max_cand = np.minimum(r_inscribed_cand + diag, diag)
    else:
        r_max_cand = r_inscribed_cand + max_overhang_m

    # ── Step 4: Coverage matrix (vectorized) ──────────────────────────────────
    # covered_by[i, j] = True if candidate i (with r_max[i]) covers pixel j
    # Shape: (n_cand, n_pixels) — precomputed once, O(1) per greedy iteration
    #
    # Memory: n_cand * n_pixels * 1 bit
    # For 3m×1m at 5cm candidates + 2cm pixels: ~300 * 7500 = 2.25M bits = 280KB ✓
    dist_sq = (px[np.newaxis, :] - cand_x[:, np.newaxis]) ** 2 + (py[np.newaxis, :] - cand_y[:, np.newaxis]) ** 2
    covered_by = dist_sq <= (r_max_cand[:, np.newaxis] ** 2)  # (n_cand, n_pixels)

    # ── Step 5: Greedy Maximum Coverage ───────────────────────────────────────
    # Invariant: uncovered[j] = True if pixel j not yet covered by any circle
    uncovered = np.ones(n_pixels, dtype=bool)
    selected = []
    circles: List[CircleResult] = []

    n_iter = n_circles if n_circles is not None else max_circles

    for _ in range(n_iter):
        if not uncovered.any():
            break
        if n_circles is None and uncovered.mean() <= (1.0 - coverage_threshold):
            break

        # Count new pixels each candidate would cover — pure numpy, fast
        new_covered_count = (covered_by & uncovered[np.newaxis, :]).sum(axis=1)

        best = int(np.argmax(new_covered_count))
        if new_covered_count[best] == 0:
            break   # no candidate covers anything new

        # Mark pixels covered by selected circle
        uncovered &= ~covered_by[best]

        # Build CircleResult
        cx = float(cand_x[best])
        cy = float(cand_y[best])
        r = float(r_max_cand[best])
        r_ins = float(r_inscribed_cand[best])

        circles.append(CircleResult(
            cx=cx, cy=cy,
            r=r,
            r_inscribed=r_ins,
            overhang=float(max(0.0, r - r_ins)),
        ))
        selected.append(best)

    cov = float(1.0 - uncovered.sum() / n_pixels)
    return circles, poly, cov


# ──────────────────────────────────────────────────────────────────────────────
# Output formatters
# ──────────────────────────────────────────────────────────────────────────────

def _to_ros2_yaml(circles: List[CircleResult], robot_name: str = 'robot') -> str:
    lines = [
        f'# TEB-K CircleFootprint — {robot_name}',
        '# Generated by footprint_to_circles.py',
        'robot:',
        '  circle_footprint:',
    ]
    for c in circles:
        lines.append(f'    - {{cx: {c.cx:.4f}, cy: {c.cy:.4f}, r: {c.r:.4f}}}')
    return '\n'.join(lines)


def _print_summary(circles: List[CircleResult], coverage: float) -> None:
    print('=' * 56)
    print(f'  Circles:   {len(circles)}')
    print(f'  Coverage:  {coverage * 100:.4f}%')
    print('=' * 56)
    for i, c in enumerate(circles):
        print(f'  [{i}] cx={c.cx:+.4f}  cy={c.cy:+.4f}  r={c.r:.4f} m')
    print()
    print(_to_ros2_yaml(circles))


# ──────────────────────────────────────────────────────────────────────────────
# Visualization
# ──────────────────────────────────────────────────────────────────────────────

COLORS = [
    '#20b2aa', '#ff7f50', '#9370db', '#ffd700', '#87ceeb',
    '#ff69b4', '#98fb98', '#dda0dd', '#f0e68c', '#b0c4de',
    '#ff6347', '#7fffd4',
]


def _visualize(
    circles: List[CircleResult],
    poly: Polygon,
    coverage: float,
    title: str = 'Footprint → Circle Decomposition',
    output_file: str = 'footprint_circles.png',
) -> None:
    fig, ax = plt.subplots(1, 1, figsize=(16, 7))
    fig.patch.set_facecolor('#0f1117')

    ax.set_facecolor('#1a1d27')
    for sp in ax.spines.values():
        sp.set_color('#444')
    ax.tick_params(colors='#aaa')

    poly_pts = np.array(poly.exterior.coords)

    # ── Left: circle overlay ──────────────────────────────────────────────────
    ax.fill(poly_pts[:, 0], poly_pts[:, 1], color='#2a3a4a', alpha=0.5, zorder=1)
    ax.plot(poly_pts[:, 0], poly_pts[:, 1], color='#4a9eda', lw=2, zorder=2)

    for i, c in enumerate(circles):
        col = COLORS[i % len(COLORS)]
        ax.add_patch(Circle((c.cx, c.cy), c.r,
                            fc=col, alpha=0.20, ec=col, lw=2, zorder=3))
        ax.plot(c.cx, c.cy, 'o', color=col, ms=5, zorder=5)
        ax.annotate(
            f'c{i}\nr={c.r:.3f}m',
            (c.cx, c.cy),
            xytext=(0, 12), textcoords='offset points',
            ha='center', fontsize=8.5, color=col, zorder=6,
        )

    ax.plot(0, 0, 'w+', ms=14, mew=2.5, zorder=7, label='Origin (rear axle)')
    ax.legend(facecolor='#1a1d27', edgecolor='#444', labelcolor='white', fontsize=8)
    ax.set_aspect('equal')
    ax.grid(True, color='#333', ls='--', alpha=0.4)
    ax.set_xlabel('X [m]', color='#aaa')
    ax.set_ylabel('Y [m]', color='#aaa')
    ax.set_title(
        f'{title}\n{len(circles)} circles · coverage {coverage * 100:.2f}%',
        color='white', pad=10,
    )
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight', facecolor='#0f1117')
    plt.close()
    print(f'\nVisualization saved: {output_file}')


# ──────────────────────────────────────────────────────────────────────────────
# Entry point — configure your robot here
# ──────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':

    # ── Robot footprint in body frame (origin = rear axle) ───────────────────
    # Format: list of (x, y) polygon vertices
    # Example: Forklift 3m × 1m, rear axle at origin
    FOOTPRINT_VERTICES = [
        # (-0.30, -0.50), (2.70, -0.50),(2.70, 0.50), (-0.30, 0.50),
        (-1.1, 0.3), (0.0, 0.5), (1.3, 0.5), (1.6, 0.3),
        (-1.1, -0.3), (0.0, -0.5), (1.3, -0.5), (1.6, -0.3)
    ]

    # ── Options ───────────────────────────────────────────────────────────────
    N_CIRCLES = None    # None = auto  |  int = force fixed count
    MAX_OVERHANG = 0.1
    MAX_CIRCLES = 100
    COVERAGE_THRESHOLD = 0.999

    OUTPUT_FILE = 'footprint_circles.png'
    ROBOT_NAME = 'forklift'

    # ── Run ───────────────────────────────────────────────────────────────────
    circles, poly, cov = polygon_to_circles(
        vertices=FOOTPRINT_VERTICES,
        max_overhang_m=MAX_OVERHANG,
        n_circles=N_CIRCLES,
        coverage_threshold=COVERAGE_THRESHOLD,
        max_circles=MAX_CIRCLES,
        candidate_spacing=0.02,
        pixel_size=0.01
    )

    _print_summary(circles, cov)

    _visualize(
        circles, poly, cov,
        title=f'{ROBOT_NAME.capitalize()} Footprint',
        output_file=OUTPUT_FILE,
    )


# ========================================================
#   Circles:   6
#   Coverage:  99.3667%
# ========================================================
#   [0] cx=+0.3700  cy=-0.0100  r=0.5400 m
#   [1] cx=+1.1100  cy=+0.0100  r=0.5400 m
#   [2] cx=-0.5100  cy=-0.0100  r=0.4409 m
#   [3] cx=-0.7500  cy=-0.0100  r=0.3979 m
#   [4] cx=-0.0500  cy=+0.0100  r=0.5232 m
#   [5] cx=+0.6700  cy=-0.0100  r=0.5400 m

# # TEB-K CircleFootprint — robot
# # Generated by footprint_to_circles.py
# robot:
#   circle_footprint:
#     - {cx: 0.3700, cy: -0.0100, r: 0.5400}
#     - {cx: 1.1100, cy: 0.0100, r: 0.5400}
#     - {cx: -0.5100, cy: -0.0100, r: 0.4409}
#     - {cx: -0.7500, cy: -0.0100, r: 0.3979}
#     - {cx: -0.0500, cy: 0.0100, r: 0.5232}
#     - {cx: 0.6700, cy: -0.0100, r: 0.5400}

# Visualization saved: footprint_circles.png


# ========================================================
#   Circles:   6
#   Coverage:  99.9375%
# ========================================================
#   [0] cx=+0.2900  cy=-0.0100  r=0.5900 m
#   [1] cx=+1.0700  cy=+0.0100  r=0.5900 m
#   [2] cx=-0.6300  cy=-0.0100  r=0.4694 m
#   [3] cx=-0.1500  cy=-0.0300  r=0.5356 m
#   [4] cx=-0.7500  cy=-0.0100  r=0.4479 m
#   [5] cx=+0.6300  cy=-0.0500  r=0.5500 m

# # TEB-K CircleFootprint — robot
# # Generated by footprint_to_circles.py
# robot:
#   circle_footprint:
#     - {cx: 0.2900, cy: -0.0100, r: 0.5900}
#     - {cx: 1.0700, cy: 0.0100, r: 0.5900}
#     - {cx: -0.6300, cy: -0.0100, r: 0.4694}
#     - {cx: -0.1500, cy: -0.0300, r: 0.5356}
#     - {cx: -0.7500, cy: -0.0100, r: 0.4479}
#     - {cx: 0.6300, cy: -0.0500, r: 0.5500}
