"""A ride change of camerad-replay --playlist (or a camera restart) resets the api's driving state."""
import json
import unittest

from roadsense_api.server import ride_line
from roadsense_api.state import State


def cam(seq, ride=None, idx=None):
    m = {"type": "camera", "seq": seq, "ts_mono_ns": seq * 50_000_000, "source": "replay"}
    if ride:
        m.update(ride=ride, ride_index=idx, ride_count=21, playlist=True, author="Road", licence="CC BY 3.0",
                 ride_t_s=60.0, ride_duration_s=900.0)
    return m


class RideChangeTest(unittest.TestCase):
    def feed(self, st, m):
        st.on_message("camerad", m, json.dumps(m))

    def test_reset_on_ride_change_and_restart(self):
        st = State("/nonexistent")
        self.feed(st, cam(1, "a", 0))
        self.feed(st, cam(2, "a", 0))
        st.speed.odo = 1234.0
        st.live_tracks[7] = {"track_id": 7, "_seen": 0}
        self.feed(st, cam(3, "a", 0))           # same ride, seq on: nothing reset
        self.assertEqual(st.speed.odo, 1234.0)
        self.assertEqual(st.ride_changes, 0)
        self.feed(st, cam(1, "b", 1))           # next ride
        self.assertEqual(st.ride_changes, 1)
        self.assertEqual(st.speed.odo, 0.0)
        self.assertEqual(st.live_tracks, {})
        st.speed.odo = 5.0
        self.feed(st, cam(10))                  # a live camera after the replay: key changes
        self.feed(st, cam(2))                   # camera restart: seq backwards
        self.assertEqual(st.ride_changes, 3)
        self.assertEqual(st.speed.odo, 0.0)

    def test_ride_line(self):
        self.assertIsNone(ride_line({"seq": 1}))
        s = ride_line(cam(1, "rural-zloczew", 1))
        self.assertTrue(s.startswith("Ride: rural-zloczew — Road, CC BY 3.0"))
        self.assertIn("(2/21, 1/15 min)", s)
        c = dict(cam(1, "s1-roadworks", 10), author="In\u017cynieria Ruchu PL. \u0141\u00f3d\u017a")
        self.assertTrue(ride_line(c, ascii_only=True).startswith("Ride: s1-roadworks - Inzynieria Ruchu PL. Lodz, CC BY"))


if __name__ == "__main__":
    unittest.main()
