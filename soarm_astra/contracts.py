"""Public seam between voice events, a snapshot planner, and local control."""
from dataclasses import dataclass
from math import isfinite
from typing import Literal, Protocol

Vec = tuple[float, ...]
TICK = 50  # milliseconds, fixture cadence
COLORS = {"blue": 1, "red": 2}
TRAY = 3
SECOND_TRAY = 4
OCCLUDED = 255


@dataclass(frozen=True)
class Intent:
    kind: Literal["goal", "override", "stop"]
    text: str = ""


@dataclass(frozen=True)
class Frame:
    camera: str
    ms: int
    width: int
    pixels: bytes

    def positions(self, color):
        return [(i % self.width, i // self.width)
                for i, value in enumerate(self.pixels) if value == color]


@dataclass(frozen=True)
class Encoder:
    ms: int
    q: Vec  # five arm joints in radians
    grip: float  # fixture calibrated jaw opening

    def valid(self):
        return len(self.q) == 5 and all(isfinite(v) for v in (*self.q, self.grip))


@dataclass(frozen=True)
class Snapshot:
    top: Frame
    wrist: Frame
    joints: Encoder
    tip: Vec

    @property
    def stamps(self):
        return self.top.ms, self.wrist.ms, self.joints.ms

    def fresh(self, now, floor):
        return (self.top.camera == "top" and self.wrist.camera == "wrist"
                and self.joints.valid() and min(self.stamps) > floor
                and 0 <= now - max(self.stamps) and now - min(self.stamps) <= 200
                and max(self.stamps) - min(self.stamps) <= 40)


class Cameras(Protocol):
    def read(self, camera: str) -> Frame: ...


class IK(Protocol):
    def forward(self, q: Vec) -> Vec: ...
    def solve(self, xyz: Vec, seed: Vec) -> Vec: ...


class Driver(Protocol):
    max_open: float
    closed: float
    def read(self) -> Encoder: ...
    def write(self, q: Vec, grip: float) -> None: ...
    def hold(self) -> None: ...
    def stop_ack(self) -> bool: ...


@dataclass(frozen=True)
class Mission:
    goal: str
    target: str
    destination: str
    subgoals: tuple[str, ...]
    success: str


@dataclass(frozen=True)
class Request:
    revision: int
    obs_id: int
    command: str
    mission: Mission | None
    phase: str
    feedback: str
    observation: Snapshot


class Planner(Protocol):
    def submit(self, request: Request) -> None: ...
    def poll(self) -> list[dict]: ...


@dataclass(frozen=True)
class Slice:
    phase: str
    points: tuple[Vec, ...]
    samples: int


@dataclass(frozen=True)
class Reply:
    decision: str
    reason: str
    mission: Mission | None
    segment: Slice | None


def keys(obj, names):
    if type(obj) is not dict or set(obj) != set(names.split()):
        raise ValueError("unexpected fields")


def nonempty(s):
    return type(s) is str and bool(s.strip())


def parse(raw):
    keys(raw, "revision obs_id decision reason mission segment")
    if any(type(raw[k]) is not int or raw[k] <= 0 for k in ("revision", "obs_id")):
        raise ValueError("invalid identity")
    decision, reason = raw["decision"], raw["reason"]
    if decision not in ("next", "retry", "replan", "review", "ask", "done") or not nonempty(reason):
        raise ValueError("invalid decision")
    mission = raw["mission"]
    if mission is not None:
        keys(mission, "goal target destination subgoals success")
        subs = mission["subgoals"]
        if (not nonempty(mission["goal"]) or mission["target"] not in COLORS
                or mission["destination"] not in ("tray", "tray_b") or mission["success"] != "target_in_tray_and_jaws_empty"
                or type(subs) is not list or not subs or not all(map(nonempty, subs))):
            raise ValueError("unsupported mission")
        mission = Mission(**{**mission, "subgoals": tuple(subs)})
    segment = raw["segment"]
    if decision in ("next", "retry", "replan"):
        keys(segment, "frame xyz_unit phase samples waypoints")
        points = segment["waypoints"]
        if (segment["frame"] != "robot_base" or segment["xyz_unit"] != "m"
                or segment["phase"] not in ("approach", "lift", "carry", "lower", "release")
                or type(segment["samples"]) is not int or not 10 <= segment["samples"] <= 20
                or type(points) is not list or not 2 <= len(points) <= 4
                or any(type(p) is not list or len(p) != 3 or
                       any(type(x) not in (int, float) or not isfinite(x) for x in p) for p in points)):
            raise ValueError("invalid Cartesian segment")
        segment = Slice(segment["phase"], tuple(map(tuple, points)), segment["samples"])
    elif segment is not None:
        raise ValueError("unexpected motion")
    if decision not in ("ask", "review") and mission is None:
        raise ValueError("missing mission")
    return Reply(decision, reason, mission, segment)
