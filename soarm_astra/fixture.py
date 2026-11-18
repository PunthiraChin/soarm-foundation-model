"""Action-dependent toy world. Camera pixels are rendered on separate simulated clocks."""
from math import dist
from .contracts import Encoder, Frame, COLORS, TRAY, OCCLUDED
from .motion import FixtureIK


class Clock:
    ms = 0


class Plant:
    max_open = .082
    closed = .012

    def __init__(self, clock, ik):
        self.clock, self.ik = clock, ik
        self.q = ik.solve((.16, .01, .10), (0, 0, 1, 0, 0))
        self.grip = self.closed
        self.target = self.q, self.grip
        self.cubes = [["blue", (.19, .065)], ["red", (.16, -.035)]]
        self.tray_xy = (.24, -.07)
        self.held = None
        self.misses = self.misplaces = 0
        self.frozen = False
        self.hold_ms = None
        self.writes = []

    def read(self):
        return Encoder(self.clock.ms, self.q, self.grip)

    def write(self, q, grip):
        self.target = q, grip
        self.writes.append((self.clock.ms, q, grip))

    def hold(self):
        # Replaces the still-active physical target in this idealised driver.
        self.hold_ms = self.clock.ms
        self.target = self.q, self.grip

    def stop_ack(self):
        return self.hold_ms is not None and self.clock.ms >= self.hold_ms + 50

    def drop(self, xy):
        if self.held is None:
            raise RuntimeError("empty gripper")
        self.cubes[self.held][1] = xy
        self.held = None

    def advance(self):
        if self.frozen:
            return
        q, grip = self.target
        self.q = tuple(a + max(-.12, min(.12, b-a)) for a, b in zip(self.q, q))
        old = self.grip
        self.grip += max(-.022, min(.022, grip-self.grip))
        tip = self.ik.forward(self.q)
        if old > .03 >= self.grip and self.held is None and tip[2] < .025:
            for i, (_, xy) in enumerate(self.cubes):
                if dist(tip[:2], xy) < .009:
                    if self.misses:
                        self.misses -= 1
                    else:
                        self.held = i
                    break
        if self.held is not None:
            self.cubes[self.held][1] = tip[:2]
            if self.grip > .045:
                xy = (tip[0]-.045, tip[1]) if self.misplaces else tip[:2]
                self.misplaces = max(0, self.misplaces-1)
                self.drop(xy)


class CameraPair:
    def __init__(self, plant):
        self.plant = plant
        self.frames = {}
        self.paused = set()
        self.obscured = set()

    def capture(self, camera, now):
        if camera in self.paused:
            return
        plant = self.plant
        if camera == "top":
            w = 72
            pixels = bytearray(w*73)
            def index(xy):
                return round(xy[0]/.005) + w*(round(xy[1]/.005)+36)
            center = index(plant.tray_xy)
            for dy in range(-4, 5):
                for dx in range(-4, 5):
                    pixels[center + dy*w + dx] = TRAY
            for color, xy in plant.cubes:
                pixels[index(xy)] = COLORS[color]
        else:
            w = 7
            pixels = bytearray(w*w)
            if plant.held is not None:
                pixels[24] = COLORS[plant.cubes[plant.held][0]]
        if camera in self.obscured:
            pixels[:] = bytes([OCCLUDED]) * len(pixels)
        self.frames[camera] = Frame(camera, now, w, bytes(pixels))

    def read(self, camera):
        return self.frames[camera]


class Bench:
    def __init__(self):
        self.clock, self.ik = Clock(), FixtureIK()
        self.plant = Plant(self.clock, self.ik)
        self.cameras = CameraPair(self.plant)
        self.advance(50)

    def advance(self, ms=50):
        for _ in range(ms):
            self.clock.ms += 1
            now = self.clock.ms
            if now % 10 == 0:
                self.plant.advance()
            if now % 25 == 0:
                self.cameras.capture("top", now)
            if now % 30 == 3:
                self.cameras.capture("wrist", now)
