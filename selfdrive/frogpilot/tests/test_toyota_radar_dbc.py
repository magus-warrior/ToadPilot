import re
import unittest
from pathlib import Path

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser

from openpilot.selfdrive.pandad import can_list_to_can_capnp


OPENPILOT_ROOT = Path(__file__).resolve().parents[3]
TOYOTA_RADAR_DBCS = ("toyota_adas", "toyota_tss2_adas")
LONG_DIST_DEFINITION = re.compile(
  r'^ SG_ LONG_DIST : 15\|15@0\+ \(0\.01,0\) \[0\|300\] "m" XXX$',
  re.MULTILINE,
)


class TestToyotaRadarDistanceSignal(unittest.TestCase):
  def test_all_track_a_slots_use_full_distance_resolution(self):
    for dbc_name in TOYOTA_RADAR_DBCS:
      with self.subTest(dbc=dbc_name):
        dbc_text = (OPENPILOT_ROOT / "opendbc" / f"{dbc_name}.dbc").read_text()
        self.assertEqual(len(LONG_DIST_DEFINITION.findall(dbc_text)), 16)
        self.assertNotIn('LONG_DIST : 15|13@0+ (0.04,0)', dbc_text)

  def test_prebuilt_dbc_matches_source_for_every_track_a_slot(self):
    # This device ships prebuilt CAN bindings. Verify the runtime DBC has the
    # same 15-bit/0.01 m definition as the source files, not merely a text edit.
    for dbc_name in TOYOTA_RADAR_DBCS:
      packer = CANPacker(dbc_name)
      for track_index in range(16):
        track_name = f"TRACK_A_{track_index}"
        with self.subTest(dbc=dbc_name, track=track_name):
          parser = CANParser(dbc_name, [(track_name, 20)], 1)
          can_msg = packer.make_can_msg(track_name, 1, {"LONG_DIST": 1.23})

          raw_long_dist = (int.from_bytes(can_msg[2][1:3], "big") >> 1) & 0x7fff
          self.assertEqual(raw_long_dist, 123)

          updated = parser.update_strings([can_list_to_can_capnp([can_msg])])
          self.assertIn(can_msg[0], updated)
          self.assertAlmostEqual(parser.vl[track_name]["LONG_DIST"], 1.23, places=2)


if __name__ == "__main__":
  unittest.main()
