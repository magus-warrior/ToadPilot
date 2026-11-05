import io
import json
import tempfile
from pathlib import Path
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
  def test_remote_refresh_keeps_only_retained_models(self):
    entries = [dict(id='wd-40', name='WD-40', version='v2'),
               dict(id='north-dakota', name='North Dakota', version='v2'),
               dict(id='not-too-shabby', name='Not Too Shabby', version='v7')]
    with patch.object(mm.urllib.request, 'urlopen', return_value=io.BytesIO(json.dumps(dict(models=entries)).encode())):
      self.assertEqual(mm.ModelManager.fetch_models('unused'), [entries[1]])
    params = MemoryParams()
    with patch.object(mm, 'params', params), patch.object(mm, 'modern_model_installed', return_value=True), \
         patch.object(mm.ModelManager, 'copy_modern_model'):
      mm.ModelManager().update_model_params(entries, None)
    self.assertEqual(params.values['AvailableModels'].split(','), ['north-dakota', *MODELS])

  def test_removed_model_cannot_be_downloaded_from_stale_request(self):
    with patch.object(mm, 'get_repository_url') as repository, patch.object(mm, 'handle_error') as error:
      mm.ModelManager().download_model('wd-40')
    repository.assert_not_called()
    error.assert_called_once()

  def test_boot_preserves_north_dakota_and_does_not_restore_old_defaults(self):
    with tempfile.TemporaryDirectory() as directory:
      models_path = Path(directory)
      fallback = models_path / 'north-dakota.thneed'
      fallback.write_bytes(b'north-dakota-artifact')
      with patch.object(mm, 'MODELS_PATH', models_path), patch.object(mm, 'modern_model_installed', return_value=False):
        mm.ModelManager.copy_default_model()
      self.assertEqual([p.name for p in models_path.iterdir()], ['north-dakota.thneed'])
      self.assertEqual(fallback.read_bytes(), b'north-dakota-artifact')

  def test_cached_catalog_adds_only_installed_models_offline_without_duplicates(self):
    params = MemoryParams()
    manager = mm.ModelManager()
    with patch.object(mm, 'params', params), patch.object(mm, 'modern_model_installed', side_effect=lambda mid: mid in {'toad-pop-v2', 'toad-south-carolina'}), \
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
