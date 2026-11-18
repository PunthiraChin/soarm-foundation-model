"""Single-writer event loop. Call intent, tick and receive on the same owner thread."""
from collections import deque
from math import dist
from .contracts import Cameras, Driver, IK, Intent, Planner, Request, Snapshot, TICK, parse
from .motion import preflight
from .vision import at_support, carried, jaws, objects, placed, separated


class ControlLoop:
    def __init__(self, clock, planner: Planner, cameras: Cameras, ik: IK, driver: Driver):
        self.clock, self.planner, self.cameras = clock, planner, cameras
        self.ik, self.driver = ik, driver
        self.rev = self.serial = 0
        self.state, self.phase = "idle", "start"
        self.pending = self.active = self.last_write = None
        self.queue = deque()
        self.log = []
        self.floor = -1
        self.retry_limit, self.deadline_ms = 2, 60000
        self.last_tick = -TICK
        self.last_encoder = -1
        self.next_goal = None

    @property
    def now(self):
        return self.clock.ms

    def event(self, name, **fields):
        self.log.append(dict(ms=self.now, revision=self.rev, event=name, **fields))

    def intent(self, intent: Intent):
        if intent.kind not in ("goal", "override", "stop"):
            raise ValueError("unsupported intent")
        if intent.kind != "stop" and not intent.text.strip():
            raise ValueError("empty goal")
        if intent.kind == "goal" and self.state not in ("idle", "done", "ask", "stopped"):
            self.event("busy")
            return
        self.event("intent", kind=intent.kind, text=intent.text)
        self.rev += 1
        if intent.kind == "goal" and self.state in ("idle", "stopped", "ask", "done"):
            self.begin(intent.text)
            return
        self.next_goal = intent.text if intent.kind == "override" else None
        if self.state == "fault":
            self.next_goal = None
            return
        if self.state != "stopping":
            self.stop("interruption")

    def begin(self, text):
        self.command, self.mission, self.phase, self.feedback = text, None, "start", "new_goal"
        self.retries = self.requests = 0
        self.first_write = True
        self.deadline = self.now + self.deadline_ms
        self.floor = self.now
        self.state = "observe"
        self.event("begin")

    def stop(self, reason):
        self.event("last_old_write", write=self.last_write)
        self.pending = self.active = None
        self.queue.clear()
        self.state, self.stop_time = "stopping", self.now
        self.acked, self.stationary = False, None
        try:
            self.driver.hold()
        except (OSError, TimeoutError, ValueError) as exc:
            self.state = "fault"
            self.event("stop_unconfirmed", reason=str(exc))
            return
        self.event("hold_sent", reason=reason)

    def fault(self, reason):
        self.next_goal = None
        self.event("fault_reason", reason=reason)
        self.stop(reason)

    def encoder(self):
        e = self.driver.read()
        if not e.valid() or not 0 <= self.now-e.ms <= 80 or e.ms < self.last_encoder:
            raise ValueError("invalid encoder")
        self.last_encoder = e.ms
        return e

    def snapshot(self):
        top = self.cameras.read("top")
        wrist = self.cameras.read("wrist")
        e = self.encoder()
        return Snapshot(top, wrist, e, self.ik.forward(e.q))

    def stopping(self):
        if self.now-self.stop_time > 1500:
            self.state = "fault"
            self.event("stop_unconfirmed")
            return
        if not self.acked:
            if self.driver.stop_ack():
                self.acked = True
                self.event("stop_ack")
            return
        e = self.encoder()
        if e.ms <= self.stop_time:
            return
        previous = self.stationary
        if previous is None or dist(e.q, previous.q) > .002 or abs(e.grip-previous.grip) > .001:
            self.stationary = e
        elif e.ms-previous.ms >= 100:
            self.event("settled", measured_ms=e.ms)
            self.floor = self.now
            self.state = "stopped"
            if self.next_goal is not None:
                goal, self.next_goal = self.next_goal, None
                self.begin(goal)

    def tick(self):
        if self.now-self.last_tick < TICK:
            return
        self.last_tick = self.now
        try:
            if self.state == "stopping":
                self.stopping()
            elif self.state in ("observe", "await", "move"):
                if self.now >= self.deadline:
                    self.fault("task_timeout")
                elif self.state == "move":
                    self.move()
                elif self.state == "observe":
                    self.observe()
                elif self.now-self.sent_ms > 2000:
                    self.fault("planner_timeout")
            for raw in self.planner.poll():
                self.receive(raw)
        except (OSError, ValueError, TimeoutError) as exc:
            if self.state == "stopping":
                self.state = "fault"
                self.event("stop_unconfirmed", reason=str(exc))
            else:
                self.fault(str(exc))

    def observe(self):
        snap = self.snapshot()
        if not snap.fresh(self.now, self.floor):
            self.event("waiting_for_fresh_frames", stamps=snap.stamps)
            return
        self.serial += 1
        self.requests += 1
        if self.requests > 48:
            self.fault("request_budget")
            return
        self.pending = Request(self.rev, self.serial, self.command, self.mission,
                               self.phase, self.feedback, snap)
        self.state, self.sent_ms = "await", self.now
        self.event("checkpoint", obs=self.serial, phase=self.phase, stamps=snap.stamps)
        self.planner.submit(self.pending)

    def refresh(self, reason):
        self.pending = None
        self.state, self.feedback, self.floor = "observe", reason, self.now

    def retry(self):
        self.retries += 1
        if self.retries > self.retry_limit:
            self.state = "ask"
            self.event("retry_exhausted")
            return False
        return True

    def receive(self, raw):
        req = self.pending
        if self.state != "await" or req is None:
            self.event("late_reply")
            return
        if isinstance(raw, dict) and ("revision" in raw and raw["revision"] != self.rev
                                     or "obs_id" in raw and raw["obs_id"] != req.obs_id):
            self.event("late_reply")
            return
        self.pending = None
        try:
            reply = parse(raw)
            if self.mission and reply.mission and self.mission != reply.mission:
                raise ValueError("mission changed without override")
        except (ValueError, TypeError, OverflowError) as exc:
            self.state = "ask"
            self.event("invalid_reply", reason=str(exc))
            return
        snap = req.observation
        if not snap.fresh(self.now, self.floor):
            self.refresh("expired_observation")
            return
        self.mission = reply.mission or self.mission
        self.event("decision", decision=reply.decision, reason=reply.reason, obs=req.obs_id)
        if reply.decision == "ask":
            self.state = "ask"
            return
        if reply.decision == "review":
            if self.retry():
                self.refresh(reply.reason)
            return
        if reply.decision == "done":
            self.state = "done" if placed(snap, self.mission) else "ask"
            self.event(self.state)
            return
        if reply.decision in ("retry", "replan") and not self.retry():
            return
        segment = reply.segment
        e = self.encoder()
        if dist(e.q, snap.joints.q) > .015 or abs(e.grip-snap.joints.grip) > .002:
            self.refresh("measured_state_changed")
            return
        try:
            self.guard(segment, snap)
            path = preflight(segment, self.ik, e)
        except ValueError as exc:
            self.event("path_rejected", reason=str(exc))
            # Also cap a faulty planner that keeps returning 'next'.
            if reply.decision in ("retry", "replan") or self.retry():
                self.refresh("path_rejected: " + str(exc))
            return
        self.queue = deque(path)
        self.active = segment
        self.goal_q, self.goal_grip = e.q, e.grip
        self.stage = "open" if segment.phase == "approach" else "path"
        self.state, self.last_target_ms = "move", self.now
        self.event("accepted", phase=segment.phase, points=segment.points, samples=len(path))

    def guard(self, segment, snap):
        phase = segment.phase
        held = jaws(snap)
        target = self.mission.target
        if held == "unknown" or len(objects(snap, target)) != 1:
            raise ValueError("uncertain object or jaws")
        if phase == "approach" and (held != "empty" or
                self.phase in ("carry", "lower") and not separated(snap, target)):
            raise ValueError("approach requires confirmed empty jaws")
        if phase == "lift" and (self.phase != "approach" or held not in ("empty", target)):
            raise ValueError("unexpected lift")
        if phase in ("carry", "lower", "release") and not carried(snap, target):
            raise ValueError("target absent from jaws")
        if phase == "release" and (not at_support(snap) or
                any(dist(p, snap.tip) > .001 for p in segment.points)):
            raise ValueError("release outside supported stationary pose")

    def write(self, q, grip):
        self.driver.write(q, grip)
        self.goal_q, self.goal_grip, self.last_target_ms = q, grip, self.now
        self.last_write = (self.rev, self.now)
        self.event("write", phase=self.active.phase, grip=grip, q=q)
        if self.first_write:
            self.event("first_new_write")
            self.first_write = False

    def move(self):
        e = self.encoder()
        if e.ms <= self.last_target_ms or dist(e.q, self.goal_q) > .008 or abs(e.grip-self.goal_grip) > .001:
            if self.now-self.last_target_ms >= 300:
                self.fault("tracking_timeout")
            return
        phase = self.active.phase
        if self.stage == "open":
            snap = self.snapshot()
            if not snap.fresh(self.now, self.floor) or jaws(snap) != "empty":
                self.active = None
                self.queue.clear()
                self.refresh("opening_withheld")
                return
            self.write(e.q, self.driver.max_open)
            self.stage = "path"
        elif self.queue:
            self.write(self.queue.popleft(), self.goal_grip)
        elif self.stage == "path" and phase in ("approach", "release"):
            if phase == "release":
                snap = self.snapshot()
                if not snap.fresh(self.now, self.last_target_ms) or not at_support(snap) or not carried(snap, self.mission.target):
                    self.refresh("release_withheld")
                    self.active = None
                    return
            self.write(e.q, self.driver.closed if phase == "approach" else self.driver.max_open)
            self.stage = "finish"
        else:
            self.phase, self.active = phase, None
            self.event("tracked", phase=phase, measured_ms=e.ms)
            self.refresh("measured_target_reached")
