"""Local path preflight and a numerical fixture for an idealised two-link arm."""
from math import acos, atan2, cos, dist, hypot, isfinite, sin


class FixtureIK:
    link = .17

    def forward(self, q):
        yaw, shoulder, elbow, _, _ = q
        reach = self.link * (cos(shoulder) + cos(shoulder + elbow))
        return (reach * cos(yaw), reach * sin(yaw),
                self.link * (sin(shoulder) + sin(shoulder + elbow)))

    def solve(self, xyz, seed):
        x, y, z = xyz
        r = hypot(x, y)
        c = (r*r + z*z) / (2*self.link**2) - 1
        if abs(c) > 1:
            raise ValueError("unreachable")
        candidates = []
        for elbow in (acos(c), -acos(c)):
            shoulder = atan2(z, r) - atan2(sin(elbow), 1 + cos(elbow))
            candidates.append((atan2(y, x), shoulder, elbow, -shoulder-elbow, seed[4]))
        return min(candidates, key=lambda q: dist(seed, q))


def preflight(segment, ik, measured):
    seed = measured.q
    start = ik.forward(seed)
    output = []
    for i, end in enumerate(segment.points):
        # Integer allocation visits every waypoint exactly, including corners.
        n = ((i + 1) * segment.samples // len(segment.points)
             - i * segment.samples // len(segment.points))
        for k in range(1, n + 1):
            p = tuple(a + (b-a)*k/n for a, b in zip(start, end))
            if not (.06 <= p[0] <= .35 and abs(p[1]) <= .18 and .012 <= p[2] <= .20):
                raise ValueError("workspace")
            q = ik.solve(p, seed)
            if (len(q) != 5 or not all(isfinite(x) and abs(x) <= 3.14 for x in q)
                    or max(abs(a-b) for a, b in zip(q, seed)) > .30
                    or dist(ik.forward(q), p) > .001):
                raise ValueError("IK residual, limit or step")
            output.append(q)
            seed = q
        start = end
    return output
