#!/usr/bin/env python3
"""Preview routes.yaml rails (map_rails_preview.png)."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import yaml

COLORS = {
    "to_0": "C0",
    "to_1": "C1",
    "to_2": "C2",
    "to_3": "C3",
    "to_kitchen": "m",
    "from_0": "C0",
    "from_1": "C1",
    "from_2": "C2",
    "from_3": "C3",
}


def _route_xy(route: dict, *, from_route: bool) -> list[tuple[float, float]]:
    pts: list[tuple[float, float]] = []

    def add(d: dict) -> None:
        pts.append((float(d["x"]), float(d["y"])))

    if from_route and "pre_dock" in route:
        add(route["pre_dock"])
    for p in route.get("spine") or []:
        add(p)
    if not from_route and "pre_dock" in route:
        add(route["pre_dock"])
    add(route["dock"])
    return pts


def _pts(route: dict, key: str) -> list[tuple[float, float]]:
    return _route_xy(route, from_route=key.startswith("from_"))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--routes",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "src/two_wheel_rails/config/routes.yaml",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "maps/restaurant/map_rails_preview.png",
    )
    args = parser.parse_args()
    with args.routes.open(encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    routes = cfg.get("routes") or {}
    fig, ax = plt.subplots(figsize=(8, 10))
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    for key, route in routes.items():
        if not key.startswith("to_"):
            continue
        pts = _pts(route, key)
        xs, ys = zip(*pts)
        c = COLORS.get(key, "k")
        ax.plot(xs, ys, "-o", color=c, label=key)
        ax.annotate(key.replace("to_", "T"), pts[-1], fontsize=9)
    k = cfg.get("kitchen") or cfg.get("spawn")
    if k:
        ax.plot(k["x"], k["y"], "s", color="gold", markersize=10)
        ax.annotate("K", (k["x"], k["y"]), fontsize=11)
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title("two_wheel_rails routes (routes.yaml)")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=120)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
