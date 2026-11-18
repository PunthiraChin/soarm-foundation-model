import math
import unittest
from copy import deepcopy
from dataclasses import replace
from unittest.mock import Mock, patch

from soarm_astra.contracts import Slice, parse
from soarm_astra.lerobot_ports import ARM, read_degree_state, send_degree_tick
from soarm_astra.motion import preflight
from soarm_astra.voice import VoiceIngress
from test_scenarios import BLUE, RED, Run


class Guards(unittest.TestCase):
    def test_malformed_proposals_fail_closed(self):
        r = Run()
        r.start()
        r.in_phase("approach")
        raw = r.p.decide(r.p.requests[0])
        for field, value in [("frame", "camera"), ("xyz_unit", "cm"), ("phase", "raw_joint"),
                             ("samples", True), ("samples", 21), ("waypoints", [[.1]*3]),
                             ("waypoints", [[.1]*3, [.1, float("nan"), .1]]),
                             ("waypoints", [[True]*3]*2), ("unexpected", 1)]:
            with self.subTest(field=field, value=value):
                bad = deepcopy(raw)
                bad["segment"][field] = value
                with self.assertRaises(ValueError):
                    parse(bad)
        r = Run()
        r.p.decide = lambda req: {"obs_id":req.obs_id, "revision":req.revision}
        r.start()
        self.assertEqual(r.finish(), "ask")
        self.assertEqual(r.b.plant.writes, [])

    def test_stale_skew_and_out_of_order_camera(self):
        r = Run()
        old = r.b.cameras.read("wrist")
        r.b.cameras.paused.add("wrist")
        r.start()
        for _ in range(8):
            r.tick()
        self.assertFalse(r.p.requests)
        r.b.cameras.paused.clear()
        r.tick()
        snap = r.p.requests[0].observation
        self.assertFalse(replace(snap, wrist=old).fresh(r.b.clock.ms, -1))
        skew = replace(snap.wrist, ms=snap.top.ms-41)
        self.assertFalse(replace(snap, wrist=skew).fresh(r.b.clock.ms, -1))
        self.assertFalse(replace(snap, wrist=replace(snap.wrist, ms=r.b.clock.ms+1)).fresh(r.b.clock.ms, -1))
        self.assertEqual(r.finish(), "done")

    def test_stale_reply_same_revision_and_old_voice_never_write(self):
        r = Run()
        voice = VoiceIngress(r.c)
        voice.deliver(2, {"kind":"goal", "text":BLUE})
        with patch.object(r.p, "poll", return_value=[]):
            r.tick()
            reply = r.p.inbox.pop()
            for _ in range(5):
                r.tick()
            r.c.receive(reply)
            self.assertEqual(r.c.feedback, "expired_observation")
            r.tick()
            r.c.receive(reply)
            self.assertEqual(r.c.state, "await")
            self.assertEqual(len(r.events("late_reply")), 1)
            voice.deliver(1, {"kind":"override", "text":RED})
            self.assertEqual(r.c.rev, 1)
            self.assertEqual(r.b.plant.writes, [])
        self.assertEqual(r.finish(), "done")

    def test_live_audio_interruption_is_not_a_stop_event(self):
        r = Run()
        voice = VoiceIngress(r.c)
        voice.deliver(1, {"kind":"goal", "text":BLUE})
        r.in_phase("approach")
        with self.assertRaises(ValueError):
            voice.deliver(2, {"kind":"audio_interrupted"})
        self.assertEqual(r.c.state, "move")
        voice.deliver(3, {"kind":"stop"})
        self.assertEqual(r.finish(), "stopped")
        self.assertTrue(r.events("hold_sent"))
        self.assertTrue(r.events("settled"))

    def test_ack_without_measured_settle_does_not_resume(self):
        r = Run()
        voice = VoiceIngress(r.c)
        voice.deliver(1, {"kind":"goal", "text":BLUE})
        r.in_phase("approach")
        voice.deliver(2, {"kind":"override", "text":RED})
        r.until(lambda: r.events("stop_ack"))
        read = r.b.plant.read
        def moving():
            e = read()
            return replace(e, q=(.1*((e.ms//50)%2), *e.q[1:]))
        with patch.object(r.b.plant, "read", moving):
            self.assertEqual(r.finish(), "fault")
        self.assertFalse(r.events("settled"))
        self.assertFalse(any(req.revision==2 for req in r.p.requests))

    def test_missing_ack_and_hold_failure(self):
        for fail in ("stop_ack", "hold"):
            with self.subTest(fail=fail):
                r = Run()
                r.start()
                r.in_phase("approach")
                fake = Mock(return_value=False) if fail == "stop_ack" else Mock(side_effect=OSError("offline"))
                with patch.object(r.b.plant, fail, fake):
                    VoiceIngress(r.c).deliver(1, {"kind":"override", "text":RED})
                    self.assertEqual(r.finish(), "fault")
                self.assertFalse(r.events("settled"))
                self.assertFalse(any(req.revision==2 for req in r.p.requests))

    def test_measured_tracking_timeout(self):
        r = Run()
        r.b.plant.frozen = True
        r.start()
        self.assertEqual(r.finish(), "stopped")
        self.assertEqual(len(r.b.plant.writes), 1)
        self.assertEqual(r.events("fault_reason")[0]["reason"], "tracking_timeout")
        self.assertFalse(r.events("tracked"))

    def test_request_timeout_retry_and_observation_deadline(self):
        r = Run()
        r.start()
        with patch.object(r.p, "poll", return_value=[]):
            self.assertEqual(r.finish(), "stopped")
        self.assertEqual(r.events("fault_reason")[0]["reason"], "planner_timeout")
        self.assertFalse(r.b.plant.writes)
        r = Run()
        r.c.deadline_ms = 500
        r.b.cameras.paused.add("top")
        r.start()
        self.assertEqual(r.finish(), "stopped")
        self.assertEqual(r.events("fault_reason")[0]["reason"], "task_timeout")
        r = Run()
        r.b.plant.misses = 10
        r.start()
        self.assertEqual(r.finish(), "ask")
        self.assertTrue(r.events("retry_exhausted"))
        self.assertFalse(r.events("done"))

    def test_false_done_requires_visual_predicate(self):
        r = Run()
        decide = r.p.decide
        def false_done(req):
            return {**decide(req), "decision":"done", "segment":None}
        r.p.decide = false_done
        r.start()
        self.assertEqual(r.finish(), "ask")
        self.assertFalse(r.b.plant.writes)

    def test_release_requires_current_support_and_held_target(self):
        r = Run()
        decide = r.p.decide
        def unsafe(req):
            raw = decide(req)
            if req.phase == "lift":
                raw["segment"]["phase"] = "release"
                raw["segment"]["waypoints"] = [list(req.observation.tip)]*2
            return raw
        r.p.decide = unsafe
        r.start()
        self.assertEqual(r.finish(), "ask")
        self.assertTrue(r.events("path_rejected"))
        self.assertIsNotNone(r.b.plant.held)
        self.assertFalse(any(e["phase"]=="release" for e in r.events("write")))

    def test_release_rechecks_visibility_immediately_before_open(self):
        r = Run()
        r.start()
        r.in_phase("release")
        r.b.cameras.obscured.add("wrist")
        self.assertEqual(r.finish(), "ask")
        self.assertIsNotNone(r.b.plant.held)
        self.assertFalse(any(e["grip"] == r.b.plant.max_open for e in r.events("write") if e["phase"]=="release"))

    def test_sample_counts_corner_hits_fk_and_seed_chain(self):
        r = Run()
        e = r.b.plant.read()
        points = ((.17,.03,.1), (.18,.01,.09), (.19,.02,.08))
        for n in (10, 12, 20):
            seg = Slice("carry", points, n)
            with patch.object(r.b.ik, "solve", wraps=r.b.ik.solve) as solve:
                path = preflight(seg, r.b.ik, e)
            self.assertEqual(len(path), n)
            self.assertEqual([call.args[1] for call in solve.call_args_list], [e.q, *path[:-1]])
            for i, point in enumerate(points, start=1):
                self.assertLess(math.dist(r.b.ik.forward(path[i*n//3-1]), point), 1e-8)

    def test_lerobot_packet_mapping_with_spy_only(self):
        follower = Mock()
        q = (.1, .2, .3, -.4, .5)
        send_degree_tick(follower, q, 83.)
        action = follower.send_action.call_args.args[0]
        self.assertEqual(len(action), 6)
        self.assertEqual(action["gripper.pos"], 83.)
        self.assertEqual([action[m+".pos"] for m in ARM], list(map(math.degrees, q)))
        follower.bus.sync_read.return_value = {m.removesuffix(".pos"):v for m,v in action.items()}
        read, grip = read_degree_state(follower)
        self.assertLess(math.dist(read,q), 1e-14)
        self.assertEqual(grip, 83.)

    def test_hold_replaces_in_flight_target(self):
        r = Run()
        r.start()
        r.in_phase("approach")
        r.until(lambda: math.dist(r.b.plant.read().q, r.b.plant.target[0]) > .001)
        before = r.b.plant.read()
        VoiceIngress(r.c).deliver(1, {"kind":"stop"})
        self.assertEqual(r.b.plant.target, (before.q, before.grip))
        self.assertEqual(r.finish(), "stopped")
        self.assertEqual(r.b.plant.q, before.q)

    def test_disagreement_between_top_and_wrist_requests_review(self):
        r = Run()
        r.start()
        r.in_phase("carry")
        read = r.b.cameras.read
        def conflicting(camera):
            frame = read(camera)
            if camera == "top":
                pixels = bytearray(frame.pixels)
                pixels[pixels.index(1)] = 0
                pixels[30*frame.width+20] = 1
                return replace(frame, pixels=bytes(pixels))
            return frame
        with patch.object(r.b.cameras, "read", conflicting):
            r.until(lambda: "conflicting camera evidence" in r.reasons())
        self.assertEqual(r.finish(), "done")
        self.assertNotIn("confirmed_drop", r.reasons())

    def test_preopen_rechecks_jaws_after_planning(self):
        r = Run()
        r.start()
        r.in_phase("approach")
        r.b.plant.held = 1
        r.tick()
        self.assertEqual(r.b.plant.writes, [])
        self.assertEqual(r.finish(), "ask")
