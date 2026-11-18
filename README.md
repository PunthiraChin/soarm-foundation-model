# SOARM-ASTRA

Design-first prototype for testing whether an unseen, single spoken instruction can drive a bounded tabletop task on a LeRobot SO-101. OpenAI only at runtime: GPT-Live handles voice and GPT-6.2 Astra interprets observations and plans. The local controller, not a cloud model, owns motion and safety.

**Status:** architecture and evaluation protocol only. No code, API requests, credentials, or hardware actions. Start with replay; the first hardware milestone is observation-only.

- [Architecture & implementation order](docs/architecture.md)
- [Evaluation protocol](docs/evaluation.md)

This is not a learned end-to-end low-level robot policy. The demo tests generalization of task interpretation, object selection, and skill sequencing over a pre-built skill set.
