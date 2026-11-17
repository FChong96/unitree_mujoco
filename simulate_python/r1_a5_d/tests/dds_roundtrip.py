#!/usr/bin/env python3
"""Run the simulator and HG client as separate processes on lo/domain 1."""
import json
import os
from pathlib import Path
import selectors
import subprocess
import sys

HERE = Path(__file__).resolve().parents[1]
OUT = HERE / 'evidence'
ROOT = HERE.parents[1]
env = os.environ.copy()
# Respect an explicit DDS installation or use this repository's setup prefix.
dds_home = env.get('CYCLONEDDS_HOME', str(ROOT / '.deps/install/cyclonedds'))
if Path(dds_home).is_dir():
    env['CYCLONEDDS_HOME'] = dds_home
    env['LD_LIBRARY_PATH'] = str(Path(dds_home) / 'lib') + (':' + env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
OUT.mkdir(exist_ok=True)
server = subprocess.Popen([sys.executable, '-u', str(HERE / 'run.py'), '--headless', '--dds',
                           '--duration', '6', '--report', str(OUT / 'dds-server.json')],
                          stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
try:
    selector = selectors.DefaultSelector()
    selector.register(server.stdout, selectors.EVENT_READ)
    if not selector.select(timeout=15):
        raise RuntimeError('Simulator did not initialize within 15 seconds')
    first = server.stdout.readline()
    if not first.startswith('DDS simulation:'):
        raise RuntimeError('Simulator startup failed: ' + first)
    client = subprocess.run([sys.executable, str(HERE / 'dds_demo.py'), '--duration', '3',
                             '--report', str(OUT / 'dds-client.json')],
                            capture_output=True, text=True, env=env, timeout=20)
    output, _ = server.communicate(timeout=20)
    (OUT / 'dds-roundtrip.log').write_text(first + output + client.stdout + client.stderr)
    if server.returncode or client.returncode:
        raise RuntimeError(f'DDS process failed: server={server.returncode}, client={client.returncode}; see evidence/dds-roundtrip.log')
    report = json.loads((OUT / 'dds-server.json').read_text())
    assert report['dds']['received'] > 100, report
    assert report['dds']['watchdog_hold'], report
    assert not report['warnings'], report
    print('PASS: separate-process HG LowCmd/LowState, CRC, right wrist, dual Dex1, timeout hold')
finally:
    if server.poll() is None:
        server.terminate()
        server.communicate(timeout=5)
