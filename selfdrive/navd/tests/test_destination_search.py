import json
from types import SimpleNamespace as NS

import pytest
from openpilot.common.params import Params
from openpilot.selfdrive.frogpilot.fleetmanager import fleet_manager, helpers
from openpilot.selfdrive.navd import navd
from openpilot.selfdrive.navd.helpers import Coordinate, coordinate_from_param


@pytest.fixture
def navigation(tmp_path, monkeypatch):
  p = Params(str(tmp_path / 'params'))
  m = Params(str(tmp_path / 'memory'))
  monkeypatch.setattr(helpers, 'params', p)
  monkeypatch.setattr(helpers, 'params_memory', m)
  monkeypatch.setattr(navd, 'Params', lambda: p)
  monkeypatch.setattr(navd, 'get_frogpilot_toggles', lambda: NS(conditional_navigation=False))
  monkeypatch.setattr(navd, 'DIRECTIONS_DIR', tmp_path)
  monkeypatch.setattr(fleet_manager, 'DIRECTIONS_DIR', tmp_path)
  monkeypatch.delenv('MAPBOX_TOKEN', raising=False)
  return p, m, fleet_manager.app.test_client()


def test_search_confirm_route_and_phone_directions(navigation, monkeypatch):
  p, m, client = navigation
  p.put('MapboxPublicKey', 'pk.test')
  p.put('MapboxSecretKey', '0')
  m.put('LastGPSPosition', '{"latitude":37,"longitude":-122}')
  start, end = [-122., 37.], [-122.001, 37.]
  name = '100 "Market" Street, Example City'
  def provider(url, params, timeout):
    assert params['access_token'] == 'pk.test'
    if 'geocode' in url:
      assert params['q'] == '100 Mar'
      assert params['autocomplete'] == 'true'
      assert params['proximity'] == '-122.0,37.0'
      data = {'features': [{'properties': {'full_address': name}, 'geometry': {'coordinates': end}}]}
    else:
      assert 'driving-traffic' in url and params['steps'] == 'true'
      data = {'routes': [{'legs': [{'steps': [{'geometry': {'coordinates': [start, end]}, 'distance': 90.,
                                               'duration': 15., 'maneuver': {'instruction': 'Head west'}}]}]}]}
    return NS(json=lambda: data, status_code=200, raise_for_status=lambda: None)

  monkeypatch.setattr(helpers.requests, 'get', provider)
  page = client.get('/addr_input')
  assert page.status_code == 200 and b'address-matches' in page.data
  response = client.post('/navigation/search', json={'query': '100 Mar'})
  assert response.status_code == 200
  assert p.get('NavDestination') is None
  place = response.json['results'][0]
  assert place['place_name'] == name
  confirmed = client.post('/nav_confirmation', data={'name': name, 'lat': place['latitude'], 'lon': place['longitude'], 'save_type': 'recent'})
  assert confirmed.status_code == 302
  assert json.loads(p.get('NavDestination'))['place_name'] == name

  messages = {}
  engine = navd.RouteEngine({}, NS(send=lambda s, msg: messages.update({s: msg})))
  engine.last_position = Coordinate.from_mapbox_tuple(start)
  engine.gps_ok = engine.localizer_valid = True
  engine.recompute_route()
  engine.send_instruction()
  assert len(messages['navRoute'].navRoute.coordinates) == 2
  assert messages['navInstruction'].valid and messages['navInstruction'].navInstruction.timeRemaining == pytest.approx(15)
  assert messages['frogpilotNavigation'].frogpilotNavigation.navigationSpeedLimit == 0
  directions = client.get('/navdirections.json').json
  assert directions['routes'][0]['Destination'] == name
  assert client.get('/CurrentStep.json').json['uuid'] == directions['uuid']
  p.remove('NavDestination')
  engine.recompute_route()
  assert not messages['navRoute'].navRoute.coordinates
  assert client.get('/CurrentStep.json').status_code == 404


def test_waze_pins_and_lookup_failures(navigation, monkeypatch):
  p, _, client = navigation
  monkeypatch.setattr(helpers.requests, 'get', lambda *args, **kwargs: pytest.fail('unexpected external lookup'))
  for text in ('37.4, -122.1', 'https://www.waze.com/ul?ll=37.4%2C-122.1&navigate=yes'):
    result = client.post('/navigation/search', json={'query': text})
    assert result.status_code == 200 and result.json['results'][0]['longitude'] == -122.1
  for query in ('', '91, 0', 'https://waze.com/ul/abc', 'No key available'):
    assert client.post('/navigation/search', json={'query': query}).status_code == 400
  assert client.post('/set_destination', json={'latitude': 99, 'longitude': 0}).status_code == 400
  assert p.get('NavDestination') is None
  p.put('NavDestination', '{')
  assert coordinate_from_param('NavDestination', p) is None


def test_tools_shows_and_updates_keys_without_disclosing_them(navigation):
  params, _, client = navigation
  params.put('MapboxPublicKey', '0')
  page = client.get('/tools')
  assert page.status_code == 200 and page.data.count(b'>Not saved</dd>') == 2
  assert b'id="edit-keys" open' in page.data and b'Backup/Restore Toggle Values' not in page.data
  assert b'driving-settings' in page.data and b'toadExperimentalMaxTorque' not in page.data

  params.put('MapboxSecretKey', 'sk.preexisting')
  response = client.post('/tools', data={'action': 'save', 'public_token': ' pk.replacement ', 'secret_token': ''}, follow_redirects=True)
  assert response.status_code == 200 and response.data.count(b'>Saved</dd>') == 2
  assert params.get('MapboxSecretKey') == b'sk.preexisting' and params.get('MapboxPublicKey') == b'pk.replacement'
  status = client.get('/mapbox_token')
  for page in (response, status):
    assert b'sk.preexisting' not in page.data and b'pk.replacement' not in page.data and page.headers['Cache-Control'] == 'no-store'
  assert status.json['public_configured'] and status.json['secret_configured']
  for path in ('/public_token_input', '/app_token_input'):
    assert b'id="edit-keys" open' in client.get(path, follow_redirects=True).data


def test_tools_distinguishes_key_rejection_and_connection_failure(navigation, monkeypatch):
  params, _, client = navigation
  monkeypatch.setenv('MAPBOX_TOKEN', 'pk.environment')
  assert b'device environment' in client.get('/tools').data
  def provider(url, params, timeout):
    assert params['access_token'] == 'pk.environment'
    if 'geocode' in url:
      return NS(status_code=401)
    raise helpers.requests.ConnectionError()
  monkeypatch.setattr(helpers.requests, 'get', provider)
  response = client.post('/tools', data={'action': 'check'})
  assert response.status_code == 200 and b'Key rejected' in response.data and b'Service unavailable' in response.data
  assert b'pk.environment' not in response.data and params.get('NavDestination') is None
