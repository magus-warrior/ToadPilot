import unittest

from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

from cereal import car
from panda import Panda

from openpilot.common.params import Params
from openpilot.selfdrive.car import gen_empty_fingerprint
from openpilot.selfdrive.car.car_helpers import enforce_toadpilot_natural_fingerprint
from openpilot.selfdrive.car.toyota.interface import CarInterface
from openpilot.selfdrive.car.toyota.values import CAR, Ecu
from openpilot.selfdrive.frogpilot.toadpilot import TOADPILOT_EPS_FIRMWARE


class TestToadPilotAuthorityPlumbing(unittest.TestCase):
  def setUp(self):
    self.params_dir = TemporaryDirectory()
    self.params = Params(self.params_dir.name)

  def tearDown(self):
    self.params_dir.cleanup()

  @staticmethod
  def eps_firmware(version=TOADPILOT_EPS_FIRMWARE):
    fw = car.CarParams.CarFw.new_message()
    fw.ecu = Ecu.eps
    fw.fwVersion = version
    return fw

  def get_params(self, candidate, *, latched, firmware_matched=True, exact_eps=True, docs=False):
    car_fw = [self.eps_firmware()] if exact_eps else [self.eps_firmware(b"different")]
    with patch("openpilot.selfdrive.car.toyota.interface.read_toadpilot_steering_response_latch",
               return_value=latched), \
         patch("openpilot.selfdrive.car.toyota.interface.toadpilot_steering_response_panda_matched",
               return_value=firmware_matched):
      return CarInterface.get_params(
        candidate, gen_empty_fingerprint(), car_fw, False, False, self.params, docs,
      )

  @staticmethod
  def authority_flag(CP):
    return any(
      int(config.safetyParam) & Panda.FLAG_TOYOTA_TOADPILOT_HIGH_TORQUE
      for config in CP.safetyConfigs
    )

  def test_exact_camry_tss2_boot_latch_sets_matching_panda_flag(self):
    stock = self.get_params(CAR.TOYOTA_CAMRY_TSS2, latched=False)
    experimental = self.get_params(CAR.TOYOTA_CAMRY_TSS2, latched=True)

    self.assertFalse(self.authority_flag(stock))
    self.assertTrue(self.authority_flag(experimental))
    self.assertEqual(
      int(experimental.safetyConfigs[0].safetyParam),
      int(stock.safetyConfigs[0].safetyParam) | Panda.FLAG_TOYOTA_TOADPILOT_HIGH_TORQUE,
    )

  def test_live_request_without_boot_latch_and_docs_stay_stock(self):
    live_only = self.get_params(CAR.TOYOTA_CAMRY_TSS2, latched=False)
    old_firmware = self.get_params(CAR.TOYOTA_CAMRY_TSS2, latched=True, firmware_matched=False)
    wrong_eps = self.get_params(CAR.TOYOTA_CAMRY_TSS2, latched=True, exact_eps=False)
    docs = self.get_params(CAR.TOYOTA_CAMRY_TSS2, latched=True, docs=True)

    self.assertFalse(self.authority_flag(live_only))
    self.assertFalse(self.authority_flag(old_firmware))
    self.assertFalse(self.authority_flag(wrong_eps))
    self.assertFalse(self.authority_flag(docs))

  def test_other_torque_angle_and_secoc_toyotas_stay_stock(self):
    candidates = (
      CAR.TOYOTA_COROLLA_TSS2,
      CAR.TOYOTA_RAV4_TSS2_2023,
      CAR.TOYOTA_RAV4_PRIME,
    )

    for candidate in candidates:
      with self.subTest(candidate=candidate):
        CP = self.get_params(candidate, latched=True)
        self.assertFalse(self.authority_flag(CP))

  def test_forced_fixed_and_fuzzy_fingerprints_strip_authority(self):
    accepted = self.get_params(CAR.TOYOTA_CAMRY_TSS2, latched=True)
    self.assertTrue(enforce_toadpilot_natural_fingerprint(
      accepted, car.CarParams.FingerprintSource.fw, True, False,
    ))
    self.assertTrue(self.authority_flag(accepted))

    rejected = (
      (car.CarParams.FingerprintSource.fw, True, True),
      (car.CarParams.FingerprintSource.fixed, True, False),
      (car.CarParams.FingerprintSource.fw, False, False),
    )
    for source, exact_match, forced in rejected:
      with self.subTest(source=source, exact_match=exact_match, forced=forced):
        CP = self.get_params(CAR.TOYOTA_CAMRY_TSS2, latched=True)
        self.assertFalse(enforce_toadpilot_natural_fingerprint(CP, source, exact_match, forced))
        self.assertFalse(self.authority_flag(CP))

    other = SimpleNamespace(carFingerprint=CAR.TOYOTA_COROLLA_TSS2, safetyConfigs=[])
    self.assertFalse(enforce_toadpilot_natural_fingerprint(
      other, car.CarParams.FingerprintSource.can, True, False,
    ))


if __name__ == "__main__":
  unittest.main()
