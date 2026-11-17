"""Name based simulation controller; no SDK or DDS imports in offline mode."""
from pathlib import Path
import math
import time

import mujoco
import numpy as np

MODEL_DIR = Path(__file__).resolve().parents[2] / 'unitree_robots/r1_a5_d'
# Sparse HG slots, never actuator index == motor slot.
SLOTS = {13: 'waist_yaw_joint',
         15: 'left_shoulder_pitch_joint', 16: 'left_shoulder_roll_joint',
         17: 'left_shoulder_yaw_joint', 18: 'left_elbow_joint', 19: 'left_wrist_roll_joint',
         22: 'right_shoulder_pitch_joint', 23: 'right_shoulder_roll_joint',
         24: 'right_shoulder_yaw_joint', 25: 'right_elbow_joint', 26: 'right_wrist_roll_joint',
         29: 'head_pitch_joint', 30: 'head_yaw_joint',
         31: 'left_dex1_finger_joint_1', 33: 'right_dex1_finger_joint_1'}
GRIP_SCALE = .041 / 5.4  # simulation assumption, metres per raw motor coordinate
WATCHDOG = .25


class Simulation:
    def __init__(self, mobile=False):
        self.model = mujoco.MjModel.from_xml_path(str(MODEL_DIR / ('scene_mobile.xml' if mobile else 'scene.xml')))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.joints = {}
        for a in range(self.model.nu):
            joint = self.model.actuator_trnid[a, 0]
            name = self.model.joint(joint).name
            self.joints[name] = (a, self.model.jnt_qposadr[joint], self.model.jnt_dofadr[joint], joint)
        self.home = {n: float(self.data.qpos[q]) for n, (_, q, _, _) in self.joints.items()}
        self.hold = self.home.copy()
        self.active_command = None
        self.last_command = -math.inf
        self.timed_out = False

    def state(self, name):
        a, q, v, _ = self.joints[name]
        return float(self.data.qpos[q]), float(self.data.qvel[v]), float(self.data.actuator_force[a])

    def accept_command(self, command, now=None):
        """command is an immutable snapshot: slot -> (mode,q,dq,tau,kp,kd)."""
        if set(command) != set(SLOTS):
            return False
        for mode, q, dq, tau, kp, kd in command.values():
            if mode not in (0, 1) or not all(math.isfinite(x) for x in (q, dq, tau, kp, kd)) or kp < 0 or kd < 0:
                return False
        self.active_command = command
        self.last_command = time.monotonic() if now is None else now
        self.timed_out = False
        return True

    def control(self, demo=False, now=None):
        now = time.monotonic() if now is None else now
        command = self.active_command
        if command is not None and now - self.last_command > WATCHDOG:
            self.hold = {n: self.state(n)[0] for n in self.joints}
            self.active_command = command = None
            self.timed_out = True
        target = self.hold.copy()
        if demo:
            target['left_wrist_roll_joint'] = .3 * math.sin(self.data.time * 1.5)
            target['right_wrist_roll_joint'] = -.3 * math.sin(self.data.time * 1.5)
            for side in ('left', 'right'):
                for finger in (1, 2):
                    target[f'{side}_dex1_finger_joint_{finger}'] = .020 + .015 * math.sin(self.data.time * 1.5)
        for name, (a, q, v, _) in self.joints.items():
            kp, kd = (12000, 500) if name.startswith('lift_') else (1500, 12) if '_dex1_' in name else (80, 4)
            # Affine actuator feedback lets implicitfast integrate velocity damping.
            self.model.actuator_biasprm[a, 1:3] = [-kp, -kd]
            self.data.ctrl[a] = self.data.qfrc_bias[v] + kp * target[name]
        if command is not None:
            for slot, name in SLOTS.items():
                mode, q, dq, tau, kp, kd = command[slot]
                a, qi, vi, joint = self.joints[name]
                scale = GRIP_SCALE if slot in (31, 33) else 1.0
                lo, hi = self.model.jnt_range[joint] / scale
                q = float(np.clip(q, lo, hi))
                drive = (tau + kp * q + kd * dq) / scale if mode else 0.
                feedback = np.array([-kp, -kd]) / scale**2 if mode else np.zeros(2)
                if slot in (31, 33):
                    # Share generalized force between the two coupled fingers.
                    other = self.joints[name[:-1] + '2'][0]
                    drive /= 2
                    feedback /= 2
                    self.data.ctrl[other] = drive
                    self.model.actuator_biasprm[other, 1:3] = feedback
                self.data.ctrl[a] = drive
                self.model.actuator_biasprm[a, 1:3] = feedback

    def step(self, demo=False, now=None):
        self.control(demo, now)
        mujoco.mj_step(self.model, self.data)
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError('Nonfinite simulation state')

    def summary(self):
        return {'time_s': round(self.data.time, 6), 'nq': self.model.nq, 'nv': self.model.nv,
                'actuators': self.model.nu, 'bodies': self.model.nbody,
                'max_abs_qvel': float(np.max(np.abs(self.data.qvel))),
                'warnings': {mujoco.mjtWarning(i).name: int(w.number) for i, w in enumerate(self.data.warning) if w.number},
                'joints': {n: self.state(n)[0] for n in self.joints}}
