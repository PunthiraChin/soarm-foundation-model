"""Grounding for indexed synthetic images with a known orthographic fixture calibration."""
from math import dist
from .contracts import COLORS, TRAY, SECOND_TRAY, OCCLUDED


def pixel_xy(pixel):
    x, y = pixel
    return x * .005, (y - 36) * .005


def objects(snap, target):
    return [pixel_xy(p) for p in snap.top.positions(COLORS[target])]


def jaws(snap):
    if OCCLUDED in snap.wrist.pixels or OCCLUDED in snap.top.pixels:
        return "unknown"
    visible = [name for name, color in COLORS.items() if color in snap.wrist.pixels]
    return visible[0] if len(visible) == 1 else ("empty" if not visible else "unknown")


def carried(snap, target):
    seen = objects(snap, target)
    return jaws(snap) == target and len(seen) == 1 and dist(snap.tip[:2], seen[0]) < .009


def tray(snap, destination="tray"):
    marker = SECOND_TRAY if destination == "tray_b" else TRAY
    pixels = list(map(pixel_xy, snap.top.positions(marker)))
    if not pixels:
        return None
    xs, ys = zip(*pixels)
    return min(xs), max(xs), min(ys), max(ys)


def in_tray(xy, box):
    return box is not None and box[0] < xy[0] < box[1] and box[2] < xy[1] < box[3]


def placed(snap, mission):
    seen = objects(snap, mission.target)
    return jaws(snap) == "empty" and len(seen) == 1 and in_tray(seen[0], tray(snap, mission.destination))


def at_support(snap, destination="tray"):
    return in_tray(snap.tip[:2], tray(snap, destination)) and abs(snap.tip[2] - .020) <= .003


def separated(snap, target):
    seen = objects(snap, target)
    return jaws(snap) == "empty" and len(seen) == 1 and dist(snap.tip[:2], seen[0]) > .012
