from types import SimpleNamespace
from unittest.mock import patch
import unittest

from openpilot.selfdrive.frogpilot.assets import model_manager as mm
from openpilot.selfdrive.toad_modeld.artifact import MODELS


class MemoryParams:
  def __init__(self):
    self.values = dict(AvailableModels='north-dakota', AvailableModelNames='North Dakota',
                       ModelVersions='v2', ExperimentalModels='')
  def get(self, key, encoding=None):
    return self.values.get(key)
  def put(self, key, value):
    self.values[key] = value


class TestLocalModelCatalog(unittest.TestCase):
  def test_cached_catalog_adds_only_installed_models_offline_without_duplicates(self):
    params = MemoryParams()
    manager = mm.ModelManager()
    with patch.object(mm, 'params', params), patch.object(mm, 'modern_model_installed', side_effect=lambda mid: mid != 'toad-rdf4'), \
         patch.object(mm.ModelManager, 'copy_modern_model'):
      manager.register_local_models()
      first = dict(params.values)
      manager.register_local_models()
    self.assertEqual(params.values, first)
    self.assertEqual(params.values['AvailableModels'].split(','), ['north-dakota', 'toad-pop-v2', 'toad-south-carolina'])
    self.assertEqual(params.values['ModelVersions'], 'v2,v11,v11')
    self.assertEqual(params.values['ExperimentalModels'], 'toad-pop-v2,toad-south-carolina')

  def test_corrupt_cached_catalog_does_not_overwrite_saved_remote_entries(self):
    params = MemoryParams()
    params.values['ModelVersions'] = 'v2,v15'
    before = dict(params.values)
    with patch.object(mm, 'params', params):
      mm.ModelManager().register_local_models()
    self.assertEqual(params.values, before)
