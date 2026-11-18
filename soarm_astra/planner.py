"""Deterministic stand-in for Astra. All geometry comes from each request's pixels and FK."""
from dataclasses import asdict
from math import dist
from .contracts import Mission
from .vision import at_support, carried, jaws, objects, placed, separated, tray


class FixturePlanner:
    def __init__(self):
        self.requests = []
        self.inbox = []

    def submit(self, request):
        self.requests.append(request)
        self.inbox.append(self.decide(request))

    def poll(self):
        replies, self.inbox = self.inbox, []
        return replies

    def decide(self, req):
        snap = req.observation
        mission = asdict(req.mission) if req.mission else None
        if mission:
            mission["subgoals"] = list(mission["subgoals"])
        def reply(decision, reason, segment=None):
            return dict(revision=req.revision, obs_id=req.obs_id, decision=decision,
                        reason=reason, mission=mission, segment=segment)
        def motion(phase, points, decision="next", reason=None):
            return reply(decision, reason or phase, dict(
                frame="robot_base", xyz_unit="m", phase=phase, samples=12,
                waypoints=[list(p) for p in points]))

        if mission is None:
            targets = [color for color, word in (("blue", "ฟ้า"), ("red", "แดง")) if word in req.command]
            if len(targets) != 1 or "ถาด" not in req.command:
                return reply("ask", "ต้องการหยิบก้อนสีอะไรไปวางที่ไหนคะ")
            boxes = [(box[0], name) for name in ("tray", "tray_b")
                     if (box := tray(snap, name)) is not None]
            if not boxes:
                return reply("ask", "ยังมองไม่เห็นถาดค่ะ")
            if len(boxes) == 2 and ("ซ้าย" in req.command) == ("ขวา" in req.command):
                return reply("ask", "ถาดซ้ายหรือขวาคะ")
            destination = (max(boxes) if "ขวา" in req.command else min(boxes))[1]
            mission = dict(goal=req.command, target=targets[0], destination=destination,
                           subgoals=["locate and grasp", "lift and verify carry", "lower and verify placement"],
                           success="target_in_tray_and_jaws_empty")
        target = mission["target"]
        held = jaws(snap)
        if held == "unknown":
            return reply("review", "occluded")
        if held not in ("empty", target):
            return reply("ask", "ยังถือก้อนเดิมอยู่ ให้พักที่ไหนคะ")
        locations, box = objects(snap, target), tray(snap, mission["destination"])
        if len(locations) != 1 or box is None:
            return reply("ask", "ระบุก้อนหรือถาดให้ชัดเจนอีกครั้งค่ะ")
        x, y = locations[0]
        tip = snap.tip
        if held == target and not carried(snap, target):
            return reply("review", "conflicting camera evidence")
        if req.phase == "approach":
            return motion("lift", [tip, (*tip[:2], .10)])
        if held == "empty":
            if placed(snap, Mission(**mission)):
                return reply("done", "target in tray and wrist empty")
            if req.phase in ("carry", "lower") and not separated(snap, target):
                return reply("review", "cannot confirm separation")
            reason = {"lift": "grasp_miss", "release": "place_miss",
                      "carry": "confirmed_drop", "lower": "confirmed_drop"}.get(req.phase, "locate")
            decision = "retry" if reason == "grasp_miss" else "next"
            if reason in ("confirmed_drop", "place_miss") or req.feedback.startswith("path_rejected"):
                decision = "replan"
            return motion("approach", [(*tip[:2], .10), (x, y, .10), (x, y, .020)],
                          decision, reason)
        tx, ty = (box[0]+box[1])/2, (box[2]+box[3])/2
        if at_support(snap, mission["destination"]):
            return motion("release", [tip, tip])
        span = dist(tip[:2], (tx, ty))
        if span > .004:
            fraction = min(1., .055 / span)
            end = (tip[0] + (tx-tip[0])*fraction, tip[1] + (ty-tip[1])*fraction, .10)
            return motion("carry", [(*tip[:2], .10), end])
        return motion("lower", [tip, (tx, ty, .020)])
