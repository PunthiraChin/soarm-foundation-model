import unittest
from copy import deepcopy
from math import dist
from unittest.mock import patch

from soarm_astra.contracts import Intent
from soarm_astra.fixture import Bench
from soarm_astra.planner import FixturePlanner
from soarm_astra.runtime import ControlLoop
from soarm_astra.vision import at_support, jaws, objects, placed

BLUE = "หยิบก้อนสีฟ้าไปวางในถาด"
RED = "ช่วยเอาก้อนแดงใส่ถาด"


class Run:
    def __init__(self):
        self.b = Bench()
        self.p = FixturePlanner()
        self.c = ControlLoop(self.b.clock, self.p, self.b.cameras, self.b.ik, self.b.plant)

    def tick(self):
        self.b.advance()
        self.c.tick()

    def until(self, predicate, ticks=1400):
        for _ in range(ticks):
            if predicate():
                return
            self.tick()
        raise AssertionError((self.c.state, self.c.log[-5:]))

    def start(self, goal=BLUE):
        self.c.intent(Intent("goal", goal))

    def finish(self):
        self.until(lambda: self.c.state in ("done", "ask", "stopped", "fault"))
        return self.c.state

    def events(self, name):
        return [event for event in self.c.log if event["event"] == name]

    def in_phase(self, phase):
        self.until(lambda: self.c.state == "move" and self.c.active.phase == phase)

    def reasons(self):
        return [event["reason"] for event in self.events("decision")]


class Scenarios(unittest.TestCase):
    def test_new_layout_single_command_visual_completion(self):
        for color, command in [("blue", BLUE), ("red", RED)]:
            with self.subTest(color=color):
                r = Run()
                r.b.plant.cubes = [["red", (.215, .025)], ["blue", (.185, -.04)]]
                r.b.plant.tray_xy = (.21, .085)
                r.start(command)
                self.assertEqual(r.finish(), "done")
                self.assertEqual(len(r.events("intent")), 1)
                final = r.p.requests[-1].observation
                self.assertTrue(placed(final, r.c.mission))
                self.assertLess(dist(objects(final, color)[0], r.b.plant.tray_xy), .005)
                self.assertIsNone(r.b.plant.held)
                self.assertEqual(r.p.requests[1].mission.goal, command)
                self.assertTrue(r.p.requests[1].mission.subgoals)
                phases = [q.phase for q in r.p.requests]
                self.assertEqual(phases[:3], ["start", "approach", "lift"])
                self.assertEqual(phases[-2:], ["lower", "release"])
                self.assertTrue(at_support(r.p.requests[-2].observation))
                self.assertEqual(len(r.events("accepted")) + 1, len(phases))
                self.assertEqual(len(r.b.plant.writes), sum(e["samples"] for e in r.events("accepted")) + 3)
                self.assertEqual(r.b.plant.writes[0][2], r.b.plant.max_open)
                self.assertEqual(r.b.plant.writes[0][1], r.p.requests[0].observation.joints.q)
                self.assertTrue(all(len(q)==5 for _, q, _ in r.b.plant.writes))

    def test_ambiguous_scene_asks_before_motion(self):
        r = Run()
        r.b.plant.cubes.append(["blue", (.17, .04)])
        r.start()
        self.assertEqual(r.finish(), "ask")
        self.assertEqual(r.b.plant.writes, [])

    def test_unreachable_last_waypoint_atomic_replan(self):
        r = Run()
        decide = r.p.decide
        def unreachable(req):
            reply = decide(req)
            if req.obs_id == 1:
                reply["segment"]["waypoints"] = [[.31, .02, .08], [.33, .02, .08]]
                reply["segment"]["samples"] = 20
            return reply
        r.p.decide = unreachable
        r.start()
        r.until(lambda: r.events("path_rejected"))
        self.assertEqual(r.b.plant.writes, [])
        self.assertEqual(r.events("path_rejected")[0]["reason"], "unreachable")
        self.assertEqual(r.finish(), "done")
        self.assertEqual(r.events("decision")[1]["decision"], "replan")

    def test_grasp_miss_retry_after_lift(self):
        r = Run()
        r.b.plant.misses = 1
        r.start()
        self.assertEqual(r.finish(), "done")
        decisions = r.events("decision")
        miss = next(d for d in decisions if d["reason"] == "grasp_miss")
        self.assertEqual(miss["decision"], "retry")
        req = next(q for q in r.p.requests if q.obs_id == miss["obs"])
        self.assertEqual(req.phase, "lift")
        self.assertEqual(jaws(req.observation), "empty")
        self.assertGreater(req.observation.tip[2], .08)
        self.assertEqual([e["phase"] for e in r.events("accepted")][:4],
                         ["approach", "lift", "approach", "lift"])

    def test_place_outside_tray_locates_new_position(self):
        r = Run()
        r.b.plant.misplaces = 1
        r.start()
        self.assertEqual(r.finish(), "done")
        self.assertIn("place_miss", r.reasons())
        approaches = [e["points"][-1][:2] for e in r.events("accepted") if e["phase"]=="approach"]
        self.assertEqual(len(approaches), 2)
        self.assertGreater(dist(*approaches), .04)
        self.assertLess(dist(approaches[1], (.195, -.07)), .005)

    def test_early_middle_and_late_drop_abort_then_relocate(self):
        for index in (1, 2, 3):
            with self.subTest(slice=index):
                r = Run()
                r.start()
                r.until(lambda: sum(e["phase"]=="carry" for e in r.events("accepted")) == index)
                r.tick()
                xy = (.145, .085)
                r.b.plant.drop(xy)
                self.assertEqual(r.finish(), "done")
                d = next(e for e in r.events("decision") if e["reason"]=="confirmed_drop")
                next_slice = next(e for e in r.events("accepted") if e["ms"] >= d["ms"])
                self.assertEqual(next_slice["phase"], "approach")
                self.assertLess(dist(next_slice["points"][-1][:2], xy), .005)
                self.assertFalse(any(e["phase"]=="release" and e["ms"] < d["ms"]
                                     for e in r.events("write")))

    def test_occlusion_holds_then_recovers_without_drop_verdict(self):
        r = Run()
        r.start()
        r.in_phase("carry")
        r.b.cameras.obscured.add("wrist")
        r.until(lambda: "occluded" in r.reasons())
        writes = len(r.b.plant.writes)
        r.tick()
        self.assertEqual(len(r.b.plant.writes), writes)
        r.b.cameras.obscured.clear()
        self.assertEqual(r.finish(), "done")
        self.assertNotIn("confirmed_drop", r.reasons())
        self.assertEqual(sum(e["phase"] == "approach" for e in r.events("accepted")), 1)

    def test_override_fences_old_response_and_resumes_from_post_stop_state(self):
        r = Run()
        r.start()
        r.in_phase("approach")
        stale = r.p.decide(r.p.requests[-1])
        for _ in range(4):
            r.tick()
        r.c.intent(Intent("override", RED))
        r.p.inbox.append(deepcopy(stale))
        r.until(lambda: bool(r.events("stop_ack")))
        count = len(r.b.plant.writes)
        r.c.receive(stale)
        self.assertEqual(len(r.b.plant.writes), count)
        # Deliver the old reply ahead of the valid reply while rev 2 is actually awaiting it.
        submit = r.p.submit
        def delayed_old(req):
            r.p.inbox.append(stale)
            submit(req)
        with patch.object(r.p, "submit", delayed_old):
            self.assertEqual(r.finish(), "done")
        ack, settled = r.events("stop_ack")[0]["ms"], r.events("settled")[0]["ms"]
        old = [e for e in r.events("write") if e["revision"] == 1]
        new = [e for e in r.events("write") if e["revision"] == 2]
        self.assertEqual(r.events("last_old_write")[0]["write"], (1, old[-1]["ms"]))
        self.assertLessEqual(old[-1]["ms"], r.events("intent")[1]["ms"])
        self.assertLess(old[-1]["ms"], ack)
        self.assertLess(ack, settled)
        self.assertLess(settled, new[0]["ms"])
        self.assertEqual(new[0]["ms"], r.events("first_new_write")[-1]["ms"])
        request = next(q for q in r.p.requests if q.revision == 2)
        self.assertGreater(min(request.observation.stamps), settled)
        self.assertGreaterEqual(len(r.events("late_reply")), 3)
        self.assertEqual(r.c.mission.target, "red")
        self.assertTrue(placed(r.p.requests[-1].observation, r.c.mission))

    def test_occupied_override_asks_and_maintains_grip(self):
        r = Run()
        r.start()
        r.in_phase("carry")
        r.tick()
        r.c.intent(Intent("override", RED))
        self.assertEqual(r.finish(), "ask")
        self.assertIsNotNone(r.b.plant.held)
        self.assertTrue(r.events("settled"))
        self.assertFalse(any(e["revision"]==2 for e in r.events("write")))
        self.assertEqual(r.b.plant.grip, r.b.plant.closed)
