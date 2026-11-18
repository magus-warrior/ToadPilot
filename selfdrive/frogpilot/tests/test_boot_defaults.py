from unittest.mock import Mock

from openpilot.common.params import Params
from openpilot.system.manager.manager import initialize_defaults


def test_boot_writes_only_changed_cache_entries_and_restores_missing_settings(tmp_path):
  live = Params(str(tmp_path / 'live'))
  cache = Mock(wraps=Params(str(tmp_path / 'cache')))
  defaults = [('IsMetric', '0'), ('Model', 'north-dakota'), ('LanguageSetting', 'en'), ('RecordFront', '0')]
  live.put('IsMetric', '0')
  cache.put('IsMetric', '0')
  live.put('Model', 'toad-pop-v2')
  cache.put('Model', 'north-dakota')
  cache.put('LanguageSetting', 'de')
  cache.reset_mock()

  initialize_defaults(live, cache, defaults, False)

  cache.put.assert_called_once_with('Model', b'toad-pop-v2')
  assert live.get('LanguageSetting') == b'de'
  assert live.get('RecordFront') == b'0'
  assert cache.get('RecordFront') is None

  initialize_defaults(live, cache, defaults, True)

  for key, default in defaults:
    assert live.get(key) == default.encode()
    assert cache.get(key) is None
