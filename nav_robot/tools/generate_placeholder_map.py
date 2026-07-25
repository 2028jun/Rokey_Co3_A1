#!/usr/bin/env python3
"""Generate a restaurant occupancy map matched to lightweight_pizza_restaurant.usda.

Layout (world / map frame):
  - Dining floor ~12 x 10 m centered at origin (walls at x=±6, y=±5)
  - Center doorway in the back wall at y=5 (|x| < 1.8)
  - Lightwheel kitchen asset translated to y≈7.44 (extends beyond dining)
  - Tables at (±3.2, -2.2) and (±3.2, 0.7); docks at (±1.82, same y)

Replace with Isaac Occupancy Map Generator output when available
(see tools/generate_occupancy_map_isaac.py and docs/OCCUPANCY_MAP.md).
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def write_pgm(path: Path, grid: np.ndarray) -> None:
    height, width = grid.shape
    path.write_bytes(
        f"P5\n{width} {height}\n255\n".encode("ascii") + grid.tobytes()
    )


def fill_rect(grid, origin_x, origin_y, res, height, width, x0, y0, x1, y1, value):
    """Fill axis-aligned rectangle in world coords (inclusive edges)."""
    c0 = int(np.floor((min(x0, x1) - origin_x) / res))
    c1 = int(np.ceil((max(x0, x1) - origin_x) / res))
    r0 = int(np.floor((min(y0, y1) - origin_y) / res))
    r1 = int(np.ceil((max(y0, y1) - origin_y) / res))
    for cy in range(r0, r1 + 1):
        for cx in range(c0, c1 + 1):
            row = height - 1 - cy
            if 0 <= cx < width and 0 <= row < height:
                grid[row, cx] = value


def clear_disk(grid, origin_x, origin_y, res, height, width, x, y, radius, wall):
    cx = int(round((x - origin_x) / res))
    cy = int(round((y - origin_y) / res))
    rad = int(round(radius / res))
    for dy in range(-rad, rad + 1):
        for dx in range(-rad, rad + 1):
            if dx * dx + dy * dy > rad * rad:
                continue
            xx, yy = cx + dx, cy + dy
            row = height - 1 - yy
            if wall <= row < height - wall and wall <= xx < width - wall:
                grid[row, xx] = 254


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "maps" / "restaurant",
    )
    parser.add_argument("--resolution", type=float, default=0.05)
    # Cover dining (-5..5) + kitchen (~7.4) with margin.
    parser.add_argument("--origin-x", type=float, default=-6.5)
    parser.add_argument("--origin-y", type=float, default=-5.5)
    parser.add_argument("--width-m", type=float, default=13.0)
    parser.add_argument("--height-m", type=float, default=15.0)
    args = parser.parse_args()

    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    res = args.resolution
    origin_x, origin_y = args.origin_x, args.origin_y
    width = int(round(args.width_m / res))
    height = int(round(args.height_m / res))
    wall = 2

    grid = np.full((height, width), 254, dtype=np.uint8)

    # Outer shell (map border)
    grid[:wall, :] = 0
    grid[-wall:, :] = 0
    grid[:, :wall] = 0
    grid[:, -wall:] = 0

    # Architecture walls from USDA (cube half-extents via scale/2 ≈ thickness).
    # Left / right walls: x=±6, y in [-5, 5]
    fill_rect(grid, origin_x, origin_y, res, height, width, -6.08, -5.0, -5.92, 5.0, 0)
    fill_rect(grid, origin_x, origin_y, res, height, width, 5.92, -5.0, 6.08, 5.0, 0)
    # Front walls y=-5 (gap near x=0 for entrance ~|x|<1.0)
    fill_rect(grid, origin_x, origin_y, res, height, width, -6.0, -5.08, -1.0, -4.92, 0)
    fill_rect(grid, origin_x, origin_y, res, height, width, 1.0, -5.08, 6.0, -4.92, 0)
    # Back walls y=5: current USDA has a 3.6 m kitchen doorway (|x|<1.8).
    fill_rect(grid, origin_x, origin_y, res, height, width, -6.0, 4.92, -1.8, 5.08, 0)
    fill_rect(grid, origin_x, origin_y, res, height, width, 1.8, 4.92, 6.0, 5.08, 0)

    # Tables (approx footprints) — centers from USDA TableSet_00..03
    tables = [
        (-3.2, -2.2),
        (3.2, -2.2),
        (-3.2, 0.7),
        (3.2, 0.7),
    ]
    for tx, ty in tables:
        fill_rect(
            grid, origin_x, origin_y, res, height, width,
            tx - 0.75, ty - 0.40, tx + 0.75, ty + 0.40, 0,
        )

    # Sideboards near corners (USDA)
    for sx, sy in [(-5.1, 3.9), (5.1, 3.9), (-5.1, -4.0), (5.1, -4.0)]:
        fill_rect(
            grid, origin_x, origin_y, res, height, width,
            sx - 0.35, sy - 0.35, sx + 0.35, sy + 0.35, 0,
        )

    # Kitchen zone (y>5): keep a center aisle free for dock/egress; mark flanks occupied.
    # Approximate counters left/right of the service aisle.
    fill_rect(grid, origin_x, origin_y, res, height, width, -6.0, 5.2, -2.0, 9.2, 0)
    fill_rect(grid, origin_x, origin_y, res, height, width, 2.0, 5.2, 6.0, 9.2, 0)
    # Far kitchen back wall
    fill_rect(grid, origin_x, origin_y, res, height, width, -6.0, 9.2, 6.0, 9.4, 0)

    # Dock clearance disks (must stay free under inflation)
    docks = [
        (-1.82, -2.20),
        (1.82, -2.20),
        (-1.82, 0.70),
        (1.82, 0.70),
        (0.21, 5.25),
        (0.0, 4.20),
        (0.0, 2.50),
        (0.0, 0.70),
    ]
    for dx, dy in docks:
        clear_disk(grid, origin_x, origin_y, res, height, width, dx, dy, 1.05, wall)

    # Keep kitchen doorway + center aisle open (wide enough for footprint+inflation)
    fill_rect(grid, origin_x, origin_y, res, height, width, -1.75, 4.5, 1.75, 7.2, 254)
    fill_rect(grid, origin_x, origin_y, res, height, width, -0.7, -3.0, 0.7, 5.5, 254)

    pgm = out_dir / "map.pgm"
    yaml_path = out_dir / "map.yaml"
    write_pgm(pgm, grid)
    yaml_path.write_text(
        (
            f"image: map.pgm\n"
            f"mode: trinary\n"
            f"resolution: {res}\n"
            f"origin: [{origin_x}, {origin_y}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.25\n"
        ),
        encoding="utf-8",
    )
    max_x = origin_x + args.width_m
    max_y = origin_y + args.height_m
    print(f"wrote {pgm}")
    print(f"wrote {yaml_path}")
    print(f"bounds: ({origin_x:.2f}, {origin_y:.2f}) .. ({max_x:.2f}, {max_y:.2f})")
    print("NOTE: approximate USDA layout; prefer Isaac omap for kitchen mesh fidelity.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
