#!/usr/bin/env python3
"""Restore pinned native sources without replacing installed code or binaries."""
import hashlib
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[2]


def restore():
  manifest = json.loads(Path(__file__).with_name('upstream_sources.json').read_text())

  def fetch(item):
    path = ROOT / item['path']
    if path.exists():
      return
    url = f"https://raw.githubusercontent.com/{manifest['repository']}/{manifest['commit']}/{item['path']}"
    for attempt in range(4):
      try:
        data = urllib.request.urlopen(url, timeout=30).read()
        digest = hashlib.sha1(f'blob {len(data)}\0'.encode() + data).hexdigest()
        if digest != item['sha']:
          raise ValueError(f"Source checksum mismatch: {item['path']}")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        return
      except Exception:
        if attempt == 3:
          raise
        time.sleep(attempt + 1)

  with ThreadPoolExecutor(max_workers=8) as pool:
    for count, _ in enumerate(pool.map(fetch, manifest['files']), 1):
      if count % 50 == 0:
        print(f"Restored {count}/{len(manifest['files'])} source files", flush=True)


if __name__ == '__main__':
  restore()
