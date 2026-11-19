"""The attribution line of a replayed ride (camerad-replay --playlist), shown wherever its frames are shown."""
import unittest

from roadsense_api.server import ride_line


def cam(seq, ride=None, idx=None):
    m = {"type": "camera", "seq": seq, "ts_mono_ns": seq * 50_000_000, "source": "replay"}
    if ride:
        m.update(ride=ride, ride_index=idx, ride_count=21, playlist=True, author="Road", licence="CC BY 3.0",
                 ride_t_s=60.0, ride_duration_s=900.0)
    return m


class RideLineTest(unittest.TestCase):
    def test_ride_line(self):
        self.assertIsNone(ride_line({"seq": 1}))
        s = ride_line(cam(1, "rural-zloczew", 1))
        self.assertTrue(s.startswith("Ride: rural-zloczew — Road, CC BY 3.0"))
        self.assertIn("(2/21, 1/15 min)", s)
        c = dict(cam(1, "s1-roadworks", 10), author="In\u017cynieria Ruchu PL. \u0141\u00f3d\u017a")
        self.assertTrue(ride_line(c, ascii_only=True).startswith("Ride: s1-roadworks - Inzynieria Ruchu PL. Lodz, CC BY"))


if __name__ == "__main__":
    unittest.main()
