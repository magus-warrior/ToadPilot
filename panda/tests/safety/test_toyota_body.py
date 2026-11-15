"""Body IO allowlist and Park/freshness boundaries, independent of driving limits."""
import itertools
import unittest

from panda import Panda
from panda.tests.libpanda import libpanda_py as lp


class TestToyotaBody(unittest.TestCase):
  TX_MSGS = None
  def setUp(self):
    self.s = lp.libpanda
    self.s.set_safety_hooks(Panda.SAFETY_TOYOTA, 73)
    self.s.init_tests()
    self.s.set_timer(100000)
    self.park()

  def rx(self, address, data, bus=0):
    return self.s.safety_rx_hook(lp.make_CANPacket(address, bus, data))

  def park(self):
    self.rx(0x3bc, bytes([0, 32, 0, 0, 0, 0, 0, 0]))
    self.rx(0xaa, bytes.fromhex('1a6f' * 4))
    self.s.set_controls_allowed(False)

  def tx(self, data, bus=0):
    return bool(self.s.safety_tx_hook(lp.make_CANPacket(0x750, bus, data)))

  def test_only_fixed_io_and_probe(self):
    for sub, duration, output in itertools.product(range(256), (0, 1, 6, 7, 255), (0, 0x20, 0x40, 0x80, 0xff)):
      expected = sub in (0x90, 0x91, 0x92, 0x93, 0xad) and ((duration == 0 and output == 0) or (1 <= duration <= 6 and (output in (0x40, 0x80) or (sub == 0xad and output == 0x20))))
      self.assertEqual(self.tx(bytes([sub, 4, 0x30, 1, duration, output, 0, 0])), expected)
    for sub in (0x90, 0x91, 0x92, 0x93, 0xad):
      self.assertTrue(self.tx(bytes([sub, 1, 0x3e, 0, 0, 0, 0, 0])))

  def test_malformed_packets_and_bus(self):
    good = bytearray([0x90, 4, 0x30, 1, 1, 0x40, 0, 0])
    for index in (1, 2, 3, 6, 7):
      bad = good.copy()
      bad[index] ^= 1
      self.assertFalse(self.tx(bad))
    self.assertFalse(self.tx(good, 1))
    self.assertFalse(self.tx(good, 2))
    self.assertFalse(self.tx(good[:-1]))

  def test_park_speed_controls_and_stale(self):
    move = bytes([0x90, 4, 0x30, 1, 1, 0x40, 0, 0])
    release = bytes([0x90, 4, 0x30, 1, 0, 0, 0, 0])
    self.assertTrue(self.tx(move))
    self.s.set_controls_allowed(True)
    self.assertFalse(self.tx(move))
    self.assertTrue(self.tx(release))
    self.park()
    self.rx(0x3bc, bytes([0, 0, 0, 0, 0, 0, 0, 0]))
    self.assertFalse(self.tx(move))
    self.park()
    # Opposite errors must not cancel out to a false stationary indication.
    self.rx(0xaa, bytes.fromhex('1a701a6e1a6f1a6f'))
    self.assertFalse(self.tx(move))
    self.park()
    self.s.set_timer(600001)
    self.assertFalse(self.tx(move))
    self.assertTrue(self.tx(release))
    self.rx(0xaa, bytes.fromhex('1a6f' * 4))
    self.assertTrue(self.tx(move))
    self.s.set_timer(2100001)
    self.rx(0xaa, bytes.fromhex('1a6f' * 4))
    self.assertFalse(self.tx(move))

  def test_reset_discards_old_park(self):
    self.s.set_safety_hooks(Panda.SAFETY_TOYOTA, 73)
    self.s.set_controls_allowed(False)
    self.assertFalse(self.tx(bytes([0x90, 4, 0x30, 1, 1, 0x40, 0, 0])))
