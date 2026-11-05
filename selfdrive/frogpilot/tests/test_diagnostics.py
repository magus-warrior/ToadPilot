import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from openpilot.selfdrive.frogpilot.diagnostics import Recorder, list_trips, read_trip, finite, driving_sample
from openpilot.selfdrive.frogpilot.fleetmanager import helpers
from openpilot.selfdrive.frogpilot.fleetmanager.fleet_manager import app


class DiagnosticsTests(unittest.TestCase):
  def test_driving_sample_uses_current_cereal_schema_and_stale_optional_inputs(self):
    from cereal import messaging
    class Snapshot(dict):
      pass
    services = ['carState', 'carControl', 'carOutput', 'controlsState', 'modelV2',
                'longitudinalPlan', 'frogpilotPlan', 'frogpilotCarState']
    snapshot = Snapshot({s: getattr(messaging.new_message(s), s) for s in services})
    snapshot.alive = {s: True for s in services}
    snapshot.valid = {s: True for s in services}
    snapshot['carState'].vEgo = 10.
    snapshot['modelV2'].modelExecutionTime = .04
    snapshot['frogpilotCarState'].trafficMode = True
    sample = driving_sample(snapshot, 10.)
    self.assertEqual(sample['speed'], 10.)
    self.assertAlmostEqual(sample['model_ms'], 40., places=4)
    self.assertTrue(sample['traffic_mode'])
    snapshot.alive['frogpilotCarState'] = False
    self.assertIsNone(driving_sample(snapshot, 11.)['traffic_mode'])

  def test_rotation_pruning_and_complete_summaries(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = Recorder(directory, segment_limit=2048, total_limit=24000, max_segments=3)
      recorder.start('test-model')
      for i in range(200):
        recorder.sample(dict(mono=i * .2, steering_pressed=i % 7 == 0,
                             requested_steer=1., output_steer=.8, model_ms=35., speed=10.))
      recorder.close()
      traces = list(Path(directory).glob('*.jsonl'))
      self.assertLessEqual(len(traces), 3)
      self.assertLessEqual(sum(p.stat().st_size for p in Path(directory).iterdir()), 24000)
      summaries = list_trips(directory)
      self.assertTrue(summaries)
      self.assertTrue(all(s['complete'] and s['samples'] > 0 for s in summaries))
      self.assertTrue(all(s['peak_model_ms'] == 35 for s in summaries))
      result = read_trip(summaries[0]['id'], directory)
      self.assertTrue(result['samples'])

  def test_override_events_count_edges_and_same_second_sessions_do_not_overwrite(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = Recorder(directory)
      recorder.start('test-model')
      first = recorder.session
      for state in [False, True, True, False, True]:
        recorder.sample(dict(mono=1., steering_pressed=state))
      recorder.close()
      self.assertEqual(recorder.summary['override_events'], 2)
      recorder.start('test-model')
      self.assertNotEqual(recorder.session, first)
      recorder.close()
      self.assertEqual(len(list(Path(directory).glob('*.jsonl'))), 2)

  def test_trace_reader_rejects_traversal_and_symlinks(self):
    with tempfile.TemporaryDirectory() as directory:
      with self.assertRaises(ValueError):
        read_trip('../private', directory)
      name = '20261105T000000Z-deadbeef-001'
      target = Path(directory) / 'other'
      target.write_text('{}')
      (Path(directory) / (name + '.jsonl')).symlink_to(target)
      with self.assertRaises(ValueError):
        read_trip(name, directory)

  def test_browser_response_is_bounded_and_nonfinite_values_are_null(self):
    with tempfile.TemporaryDirectory() as directory:
      recorder = Recorder(directory)
      recorder.start('test-model')
      for i in range(2200):
        recorder.sample(dict(mono=i * .2, speed=finite(float('nan'))))
      recorder.close()
      response = read_trip(recorder.summary['id'], directory)
      self.assertLessEqual(len(response['samples']), 1000)
      self.assertIsNone(response['samples'][0]['speed'])

  def test_model_changes_require_confirmed_offroad_and_installed_runtime(self):
    with patch.object(helpers, '_require_toadpilot_offroad', side_effect=ValueError('onroad')), \
         patch.object(helpers.params, 'put') as put:
      with self.assertRaises(ValueError):
        helpers.set_toadpilot_model({'model': 'toad-south-carolina'})
      put.assert_not_called()
    with patch.object(helpers, '_require_toadpilot_offroad'), \
         patch.object(helpers, 'get_toadpilot_models', return_value={'models': [dict(id='missing', installed=False)]}):
      with self.assertRaises(ValueError):
        helpers.set_toadpilot_model({'model': 'missing'})

  def test_http_health_reports_stale_and_download_rejects_invalid_id(self):
    client = app.test_client()
    with patch.object(helpers.params_memory, 'get', return_value=json.dumps({'published_at_unix': 1}).encode()):
      response = client.get('/toadpilot/health')
      self.assertEqual(response.status_code, 200)
      self.assertFalse(response.json['fresh'])
      self.assertEqual(response.headers['Cache-Control'], 'no-store')
    self.assertEqual(client.get('/toadpilot/trips/invalid/download').status_code, 404)
    self.assertEqual(client.get('/diagnostics').status_code, 200)
    self.assertEqual(client.post('/toadpilot/models', json={'model': 'unknown', 'extra': True}).status_code, 400)


if __name__ == '__main__':
  unittest.main()
