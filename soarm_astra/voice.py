"""In-process callback boundary for typed voice delegations, exercised without a microphone."""
from .contracts import Intent


class VoiceIngress:
    def __init__(self, loop):
        self.loop = loop
        self.sequence = -1

    def deliver(self, sequence, payload):
        # Sequence is assigned at source, before asynchronous delivery.
        if type(sequence) is not int or sequence < 0 or not isinstance(payload, dict):
            raise ValueError("invalid voice event")
        kind = payload.get("kind")
        if kind == "stop" and set(payload) == {"kind"}:
            intent = Intent("stop")
        elif (kind in ("goal", "override") and set(payload) == {"kind", "text"}
              and isinstance(payload["text"], str) and payload["text"].strip()):
            intent = Intent(kind, payload["text"])
        else:
            raise ValueError("invalid voice delegation")
        if sequence <= self.sequence:
            self.loop.event("old_voice_event", sequence=sequence)
            return
        self.sequence = sequence
        self.loop.intent(intent)
