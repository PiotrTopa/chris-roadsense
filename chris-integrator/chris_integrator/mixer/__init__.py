"""The mixer: input channels, the bus, plug-in transformers, output channels (docs/DESIGN.md section 14).

    channel in --> BUS (named streams, time-stamped envelopes) --> channel out
                    ^   |
                    |   v
              plug-in transformers (declared inputs -> declared outputs)

Nothing in this package knows a sensor, a tracker or a consumer."""
from .bus import Bus, Envelope, LiveClock, MessageClock, StreamSpec
from .mixer import Mixer, MixerError
from .plugin import In, Out, Plugin

__all__ = ["Bus", "Envelope", "StreamSpec", "LiveClock", "MessageClock", "Mixer", "MixerError", "In", "Out", "Plugin"]
