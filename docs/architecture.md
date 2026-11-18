# SOARM-ASTRA control contract

The offline entry point connects VoiceIngress, ControlLoop, FixturePlanner, independently clocked cameras, an idealised arm and a numerical IK fixture in one process. The intended live components are OpenAI GPT Live for voice delegation and OpenAI Astra for planning and verification. Microphone transport, network inference and physical drivers have not been exercised.

## Ownership and sequencing

1. A voice callback delivers a source-ordered typed goal, override or stop on the loop owner thread. Audio-stream interruption alone has no motor effect. An independent local stop can call `intent(Intent("stop"))` directly.
2. The loop assigns a revision, captures independently timestamped top, wrist and measured five-joint/gripper state, then submits one planner request. A real planner transport must enqueue work without blocking the loop and return responses via `poll`. Replies are fenced by both revision and observation ID. A late or stale response cannot start a slice.
3. Astra owns the interpreted goal, subgoals, task-level success condition and decisions. The fixture chooses target geometry from each new observation. The loop retains the mission for the revision, reports the tracked phase and reason for rejection, and enforces local evidence and motion guards. A new revision starts a new mission.
4. Each motion reply specifies a phase, 2–4 Cartesian points, frame `robot_base`, XYZ in metres and 10–20 samples. The phase labels govern gripper/checkpoint constraints. Coordinates and the route come from the planner. Unknown fields, invalid counts, nonfinite coordinates and unsupported units are rejected. In this fixture the approach slice closes the jaws at its tracked endpoint. Release requires a stationary path at a verified support.
5. The executor interpolates from measured FK through every corner and solves IK for every sample. The first seed is the measured arm state and each solution seeds the following solve. It checks workspace, joint bounds, step sizes and FK residual for the entire slice before any write, including pre-open. The local open command holds measured q and waits for measured calibrated maximum opening before approach or regrasp. The path runs at a nominal 50 ms interval, 0.5–1 s for the samples alone. Observation, pre-opening, gripper actions, tracking and planner time add to the total.
6. Five arm positions in radians and a separate gripper channel are written per tick. The next target waits for measured readback. The loop checks again after grasp, after lift, at every carry boundary, before release and after placing. Paired visual evidence distinguishes empty, held, uncertain and placed. Absence of evidence requests another view. Holding a different object after an override leads to a question with grip maintained.
7. Stop increments revision, fences replies, discards remaining samples and calls a local driver hold that replaces its active target. ACK and stable encoder measurements are separate transitions. The fixture requires successive stationary readings spanning 100 simulated ms after ACK. The new task requires all sensor timestamps strictly later than settling. The trace records intent, last old write, hold, ACK, settled and first new write. Missing confirmation latches a fault.

For example, two planner waypoints in a carry slice can be expressed as:

```json
{"frame":"robot_base","xyz_unit":"m","phase":"carry","samples":12,"waypoints":[[0.19,0.065,0.10],[0.21,0.015,0.10]]}
```

The complete response also has revision, obs_id, decision, reason and mission. See `soarm_astra/contracts.py` for the validated wire contract. Decisions include next, retry, replan, review, ask and done. In the fixture two bounded recovery attempts, a request cap and a task deadline prevent unbounded loops. The 200 ms age and 40 ms inter-source skew budgets are test settings. A live planner with longer latency will need observation revalidation and measured budgets.

## Substitution and fidelity

| Interface | Exercised path | Work for a live system |
|---|---|---|
| Voice and planner | Typed callback and deterministic Thai fixture planner | Connect GPT Live audio and bridge client delegation. Supply Astra labelled camera stills, measured state, mission and previous outcome. Validate its response with the same local parser. |
| Observation | Two independently rendered indexed images, measured state and fixture FK. An orthographic fixture mapping localises a visible cube and tray. | Record capture and encoder times in a common clock domain, calibrate both cameras, extrinsics and tabletop, validate object localisation, support height and carry evidence. Implement real-image grounding for the predicates in vision.py. A recorded Cameras implementation can drive replay, but a fixed recording cannot react to a changed trajectory. |
| IK | Idealised two-link position model, yaw, compensating wrist and roll. Seed chooses elbow branch. | Supply commissioned SO-101 URDF, joint order, tip frame, orientation constraints, calibrated limits, collision and tracking checks. PlacoIK only adapts units/target format for a position-only scaffold. |
| Driver | Rate-limited synthetic joint/gripper readback. Explicit mock hold and delayed ACK. | Validate serial readback rates, joint/gripper units, timestamps, tracking, slip, stop/settle, watchdog and a supervised physical stop. The fixture thresholds and jaw values are not hardware calibration. |

The packet shim in `lerobot_ports.py` expects a previously configured degree-mode follower. It maps one tick to `send_action`, which can clip commands, and has a separate encoder read. There is no hardware entry point in the demo. A production adapter must also provide independently timestamped camera capture and a commissioned hold/ACK implementation. Official SOFollower reads encoders and then collects the cameras sequentially. A returned dictionary or action is insufficient evidence of synchronized exposure, achieved rate or motion completion. [LeRobot SOFollower source](https://github.com/huggingface/lerobot/blob/main/src/lerobot/robots/so_follower/so_follower.py)

LeRobot RobotKinematics accepts a URDF, joint order and tip frame, takes joint seeds in degrees and returns degrees. The scaffold converts radians explicitly; a URDF/Placo solve has not been run here. Verify and pin the installed LeRobot revision before commissioning. [RobotKinematics source](https://github.com/huggingface/lerobot/blob/main/src/lerobot/model/kinematics.py) · [Official action representations](https://huggingface.co/docs/lerobot/action_representations)

The earlier design and fixed-skill baseline are preserved in Git at `d0c6388`.
