#!/usr/bin/env python3
"""A simulation-only HG client: wrist motion and independent Dex1 targets."""
import argparse
import json
import math
from pathlib import Path
from threading import Lock
import time

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
from dds_bridge import initialize_dds, MODE_MACHINE
from simulation import SLOTS


def main():
    parser = argparse.ArgumentParser(description='Simulation-only DDS client: lo / domain 1')
    parser.add_argument('--duration', type=float, default=5)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be finite and positive')
    initialize_dds()
    crc = CRC()
    lock = Lock()
    samples = []
    invalid = 0

    def receive(state):
        nonlocal invalid
        if state.crc != crc.Crc(state):
            invalid += 1
            return
        with lock:
            samples.append(state)

    sub = ChannelSubscriber('rt/lowstate', LowState_)
    pub = ChannelPublisher('rt/lowcmd', LowCmd_)
    sub.Init(receive, 1)
    pub.Init()
    try:
        deadline = time.monotonic() + 8
        while not samples and time.monotonic() < deadline:
            time.sleep(.02)
        if not samples:
            raise RuntimeError('No simulator LowState on lo/domain 1; start run.sh --dds')
        with lock:
            initial = samples[-1]
        if initial.mode_machine != MODE_MACHINE:
            raise RuntimeError('Unexpected simulator mode_machine')
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_machine = MODE_MACHINE
        for slot in SLOTS:
            m = cmd.motor_cmd[slot]
            m.mode, m.q, m.tau = 1, initial.motor_state[slot].q, initial.motor_state[slot].tau_est
            m.kp, m.kd = (5., .05) if slot in (31, 33) else (80., 4.)
        sent = 0
        start = time.monotonic()
        while (elapsed := time.monotonic() - start) < args.duration:
            envelope = min(1., elapsed / .5)
            cmd.motor_cmd[26].q = initial.motor_state[26].q + .2 * envelope * math.sin(elapsed * 2)
            for slot, phase in [(31, 0.), (33, math.pi)]:
                goal = 2.5 + 1.7 * math.sin(elapsed * 1.5 + phase)
                cmd.motor_cmd[slot].q = initial.motor_state[slot].q + envelope * (goal - initial.motor_state[slot].q)
                cmd.motor_cmd[slot].tau = 0.
            cmd.crc = crc.Crc(cmd)
            pub.Write(cmd)
            sent += 1
            time.sleep(.005)
        with lock:
            snapshot = samples.copy()
        report = {'interface': 'lo', 'domain': 1, 'sent': sent, 'received': len(snapshot),
                  'invalid_state_crc': invalid,
                  'ranges': {str(slot): [min(s.motor_state[slot].q for s in snapshot),
                                        max(s.motor_state[slot].q for s in snapshot)] for slot in [19, 26, 31, 33]}}
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
        if len(snapshot) < 20 or invalid or report['ranges']['26'][1] - report['ranges']['26'][0] < .1:
            raise RuntimeError('DDS feedback did not show expected wrist motion')
        for slot in ('31', '33'):
            if report['ranges'][slot][1] - report['ranges'][slot][0] < 1.:
                raise RuntimeError(f'DDS feedback did not show gripper {slot} motion')
    finally:
        sub.Close()
        pub.Close()


if __name__ == '__main__':
    main()
