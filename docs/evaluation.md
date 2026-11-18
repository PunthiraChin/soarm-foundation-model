# One-command tabletop evaluation

## Offline scenario matrix

Run `python3 -B -m unittest discover -s tests -v` and `python3 -B -m soarm_astra --trace` from the repository root. These tests exercise a deterministic feedback loop with synthetic raster images, action-dependent scene updates, fixture kinematics and a fake clock. Generated images and events stay in memory.

| Case | Injected condition | Asserted observable outcome |
|---|---|---|
| 1. New layout and Thai goal | Reposition both colours and the tray, select blue or red | Exactly one goal. Final top image places the requested cube inside the relocated tray and wrist shows empty jaws. Checkpoints follow grasp, lift, carry, lower and release. |
| 2. Ambiguity | Two visible blue cubes, or one cube with two candidate trays | Ask before the first write. A revision-bound answer selects left/right and completes the same task using fresh images. |
| 3. Unreachable IK | Final Cartesian corner exceeds fixture reach within the configured rectangular workspace | Reject the full path with zero writes including pre-open. Replan and finish. |
| 4. Grasp miss | First closure fails to attach the target | After lift, wrist is empty and target remains visible. Re-open on retry and grasp before carrying. |
| 5. Missed tray | Release deposits the cube outside the tray | Post-place verification rejects success. Locate and approach the new position, then confirm placement. |
| 6. In-flight loss | Drop in early, middle or final carry slice. Separately obscure a view or contradict wrist evidence | At the next boundary, empty wrist plus top separation invalidates the remaining carry. Re-approach the observed drop position. Occlusion or disagreement requests new evidence while stationary. |
| 7. Override | Blue → red during approach, old response delayed beyond ACK. Separate occupied-gripper override during carry | Replace old setpoint, wait for ACK and measured settle, require all three sensor timestamps after settle. Old reply produces no write. Empty hand completes red task. Occupied hand holds and asks. |

The guard suite also checks malformed proposals, stale/skewed/out-of-order input, timeout and retry budgets, false completion, seed chaining, release gating, failed stop/settle and degree-mode packet mapping with a spy. These checks exercise code paths and observable fixture outcomes. Absolute response rates, slip probability, voice quality, model task generalization and physical stop distance require other evidence.

Example override trace on the fixture clock: last old write and override intent at 300 ms (write ordered first), hold at 300, ACK at 350, measured settle at 500 and first new write at 600. The default blue run uses eight planner checkpoints and 87 writes. A trace can include stationary gripper writes in addition to the Cartesian samples.

## Claim for a later live pilot

Test whether one previously unseen Thai utterance can produce the requested tabletop outcome in a supported scene without additional training or examples. Count first-attempt and recovered successes separately. The offline fixture has a restricted vocabulary and does not measure model performance. A replay is useful for scoring recognition against fixed frames; it cannot measure how a different executed path changes later observations.

## Pilot suite

Freeze the planner, waypoint contract, object labels and success conditions before evaluation. Use harmless objects (colored cubes or tokens) and fixed trays, with a clear workspace. Before exposing individual held-out prompts or layouts to the planner, write them down for later replay.

- **Known skill, new layout:** rearranged target/decoy starting positions; ask for a new object–destination pairing.
- **New wording in Thai:** refer by color or a supported spatial relation (e.g. left of another object); use the same scene for a text-only replay to separate voice recognition issues.
- **New composition:** a single utterance names two sequential transfers with different destinations. Both independent outcomes and order matter if order is specified.
- **Ambiguity / interruption:** two plausible matches, missing object, or user says stop/changes aim mid-run. Correct response may be asking or refusing to move, not completing.
- Start with five held-out trials in each of the three completion categories. Report counts, not broad performance claims from a small pilot. Report negative tests separately.

## Measurement

For each trial, log model and prompt versions, task/observation IDs, case type, ground-truth target and destination, success without recovery, success after bounded recovery, operator interventions, reasons for failure, time to first action/end, request count and actual usage. Report voice intent, grounding, planner errors, execution errors and verification separately. Compare against a simple deterministic command/skill baseline; compare to Fable quantitatively only if the same inputs, conditions and scoring are available. Independent scorer (human or validated local tracking): correct target and final tray, plus any specified order; “done” from a model does not count. Record stop behavior and false-success count.

## Public benchmark context (checked 2026-11-18)

Anthropic reports 64.9% for Claude Fable 5.2 max on **RoboDojo Sim**, versus 29.8% for **GPT-6 Astra**, an earlier model, in Anthropic's own harness. The simulated score and the separate qualitative real-world robot videos are different evidence. Neither is a matched test of an SO-101. The GPT-6.2 Astra public release and system card reviewed do not report an equivalent robotic manipulation/robot-dataset benchmark; BenchCAD is CAD, not manipulation. This is a finding about their public evaluation disclosures, not about the underlying capability or training data.

- [Anthropic Fable 5.2, RoboDojo Sim and physical demos](https://www.anthropic.com/claude-fable-and-mythos-5-2)
- [RoboDojo: simulated and real-world splits](https://robodojo-benchmark.com/)
- [OpenAI GPT-6.2 launch and linked System Card](https://openai.com/index/introducing-gpt-6-2-astra-sol/)
