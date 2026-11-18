# One-command tabletop evaluation

## Claim we can test

“Without new task-specific training or in-context demos, the system interprets one unseen spoken instruction and sequences known tabletop skills to achieve the requested outcome.” This is **one instruction / zero-shot task generalization** within a supported workspace and skill set, not one-shot learning from a demonstration, novel motor-skill acquisition or general-purpose robotics. Closed-loop recovery can follow the single instruction; do not label a recovery as first-attempt success.

## Pilot suite

Freeze the planner, skills, object labels and success conditions before evaluation. Use harmless objects (colored cubes or tokens) and fixed trays, with a clear workspace. Before exposing individual held-out prompts or layouts to the planner, write them down for later replay.

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
