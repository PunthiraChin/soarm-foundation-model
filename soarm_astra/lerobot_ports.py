"""Uncommissioned adapter seams. Nothing here opens a port or enables an arm.

The tick bridge expects an already configured degree-mode follower and a gripper
value in its configured units. Camera capture clocks, encoder timestamps and a
tested local hold/ACK must be supplied separately before a real Driver exists.
"""
from math import degrees, radians


ARM = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")


def send_degree_tick(follower, q_rad, gripper_native):
    action = {name + ".pos": degrees(value) for name, value in zip(ARM, q_rad, strict=True)}
    action["gripper.pos"] = gripper_native
    return follower.send_action(action)


def read_degree_state(follower):
    values = follower.bus.sync_read("Present_Position")
    return tuple(radians(values[name]) for name in ARM), values["gripper"]


class PlacoIK:
    """Inject LeRobot RobotKinematics with commissioned 5-joint order and tip frame."""
    def __init__(self, model):
        self.model = model

    def forward(self, q):
        import numpy as np
        pose = self.model.forward_kinematics(np.rad2deg(q))
        return tuple(pose[:3, 3])

    def solve(self, xyz, seed):
        import numpy as np
        pose = np.eye(4)
        pose[:3, 3] = xyz
        solution = self.model.inverse_kinematics(np.rad2deg(seed), pose, orientation_weight=0.0)
        return tuple(np.deg2rad(solution))
