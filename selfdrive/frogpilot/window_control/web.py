"""Fleet Manager page and authenticated bridge for browser/PWA clients."""
import re
import secrets
from urllib.parse import urlsplit

import requests
from flask import jsonify, render_template, request, session

from .server import TOKEN_PATH


def register(app):
  @app.get('/vehicle-controls')
  def vehicle_controls():
    if 'body_csrf' not in session:
      session['body_csrf'] = secrets.token_urlsafe(32)
    response = app.make_response(render_template('vehicle_controls.html', body_csrf=session['body_csrf']))
    response.headers['Cache-Control'] = 'no-store'
    response.headers['X-Frame-Options'] = 'DENY'
    return response

  @app.route('/api/vehicle/<path:resource>', methods=['GET', 'POST'])
  def vehicle_api(resource):
    allowed = ((request.method == 'GET' and (resource in ('v1/windows', 'openapi.json') or re.fullmatch(r'v1/jobs/[a-f0-9]{32}', resource))) or
               (request.method == 'POST' and resource in ('v1/window-jobs', 'v1/stop')))
    if not allowed:
      return jsonify(error='Unknown vehicle endpoint'), 404
    try:
      token = TOKEN_PATH.read_text().strip()
    except OSError:
      return jsonify(error='Vehicle API unavailable'), 503
    supplied = request.headers.get('Authorization', '')
    authenticated = secrets.compare_digest(supplied, 'Bearer ' + token)
    if not authenticated:
      csrf = session.get('body_csrf', '')
      if not csrf or not secrets.compare_digest(request.headers.get('X-Toad-CSRF', ''), csrf):
        return jsonify(error='Open Vehicle controls first or supply a bearer token'), 401
      origin = request.headers.get('Origin')
      if origin and urlsplit(origin).netloc != request.host:
        return jsonify(error='Same-origin request required'), 403
      if request.headers.get('Sec-Fetch-Site') == 'cross-site':
        return jsonify(error='Same-origin request required'), 403
    if request.content_length and request.content_length > 16384:
      return jsonify(error='Request too large'), 413
    if request.method == 'POST' and not request.is_json:
      return jsonify(error='JSON required'), 400
    try:
      upstream = requests.request(request.method, 'http://127.0.0.1:8766/' + resource,
                                  headers={'Authorization': 'Bearer ' + token},
                                  json=request.get_json() if request.method == 'POST' else None,
                                  timeout=2.)
      response = app.response_class(upstream.content, status=upstream.status_code, mimetype='application/json')
      response.headers['Cache-Control'] = 'no-store'
      return response
    except requests.RequestException:
      return jsonify(error='Vehicle API is restarting or unavailable'), 503
