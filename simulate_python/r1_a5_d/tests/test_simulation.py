import math
from pathlib import Path
import sys
import unittest

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from build_model import resolved_source
from simulation import Simulation, SLOTS, GRIP_SCALE, WATCHDOG


def transform(xyz, rpy):
    r, p, y = map(float, rpy.split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    t = np.eye(4)
    t[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                 [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    t[:3, 3] = list(map(float, xyz.split()))
    return t


class ModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sim = Simulation()

    def setUp(self):
        self.sim = self.__class__.sim
        mujoco.mj_resetDataKeyframe(self.sim.model, self.sim.data, 0)
        mujoco.mj_forward(self.sim.model, self.sim.data)
        self.sim.active_command = None
        self.sim.hold = self.sim.home.copy()

    def test_geometry_inertia_axes_and_limits_match_source(self):
        source, _ = resolved_source()
        sim = self.sim
        children = {}
        for joint in source.findall('joint'):
            children.setdefault(joint.find('parent').get('link'), []).append(joint)
        poses = {'world': np.eye(4)}
        # Check a nonzero configuration; this catches RPY convention and joint-axis errors.
        for i in range(sim.model.njnt):
            lo, hi = sim.model.jnt_range[i]
            sim.data.qpos[sim.model.jnt_qposadr[i]] = .37*lo + .63*hi if sim.model.jnt_limited[i] else .21
        mujoco.mj_forward(sim.model, sim.data)

        def visit(parent):
            for joint in children.get(parent, []):
                child = joint.find('child').get('link')
                o = joint.find('origin')
                pose = transform(o.get('xyz', '0 0 0'), o.get('rpy', '0 0 0')) if o is not None else np.eye(4)
                if joint.get('type') != 'fixed':
                    mjj = sim.model.joint(joint.get('name'))
                    axis = np.array(list(map(float, joint.find('axis').get('xyz').split())))
                    np.testing.assert_allclose(mjj.axis, axis, atol=1e-12)
                    q = sim.data.qpos[mjj.qposadr[0]]
                    motion = np.eye(4)
                    if joint.get('type') == 'prismatic':
                        motion[:3, 3] = q*axis
                    else:
                        x, y, z = axis
                        cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                        motion[:3, :3] += math.sin(q)*cross + (1-math.cos(q))*(cross@cross)
                    pose = pose @ motion
                    limit = joint.find('limit')
                    if limit is not None and limit.get('lower') is not None:
                        np.testing.assert_allclose(mjj.range, [float(limit.get('lower')), float(limit.get('upper'))])
                    if joint.get('name') in sim.joints:
                        a = sim.joints[joint.get('name')][0]
                        np.testing.assert_allclose(sim.model.actuator_forcerange[a], [-float(limit.get('effort')), float(limit.get('effort'))])
                poses[child] = poses[parent] @ pose
                b = sim.model.body(child)
                np.testing.assert_allclose(sim.data.xpos[b.id], poses[child][:3, 3], atol=2e-9)
                np.testing.assert_allclose(sim.data.xmat[b.id].reshape(3, 3), poses[child][:3, :3], atol=2e-9)
                visit(child)
        visit('world')
        for link in source.findall('link'):
            inertial = link.find('inertial')
            if inertial is None:
                continue
            b = sim.model.body(link.get('name'))
            self.assertAlmostEqual(b.mass[0], float(inertial.find('mass').get('value')), places=10)
            np.testing.assert_allclose(b.ipos, list(map(float, inertial.find('origin').get('xyz').split())), atol=1e-12)
            rotation = np.zeros(9)
            mujoco.mju_quat2Mat(rotation, b.iquat)
            rotation = rotation.reshape(3, 3)
            actual = rotation @ np.diag(b.inertia) @ rotation.T
            v = inertial.find('inertia').attrib
            expected = np.array([[float(v['ixx']), float(v['ixy']), float(v['ixz'])],
                                 [float(v['ixy']), float(v['iyy']), float(v['iyz'])],
                                 [float(v['ixz']), float(v['iyz']), float(v['izz'])]])
            # MuJoCo's inertia diagonalization uses an approximate eigensolver.
            self.assertLess(np.linalg.norm(actual - expected), 1e-6 * np.linalg.norm(expected))
        for side in ('left', 'right'):
            self.assertEqual(sum(n.startswith(side + '_') and '_dex1_' not in n for n in sim.joints), 5)
        self.assertEqual(sim.model.neq, 3)
        self.assertEqual(set(SLOTS), {13, 15, 16, 17, 18, 19, 22, 23, 24, 25, 26, 29, 30, 31, 33})

    def command(self, left=.7, right=.7):
        sim = self.sim
        command = {}
        for slot, name in SLOTS.items():
            a, q, v, _ = sim.joints[name]
            command[slot] = (1, sim.data.qpos[q], 0., sim.data.qfrc_bias[v], 80., 4.)
        command[31] = (1, left, 0., 0., 5., .05)
        command[33] = (1, right, 0., 0., 5., .05)
        return command

    def test_dual_gripper_open_close_mimic_and_independence(self):
        sim = self.sim
        for left, right in [(4.8, .7), (.7, 4.8), (.7, .7)]:
            self.assertTrue(sim.accept_command(self.command(left, right), now=0))
            for _ in range(500):
                sim.step(now=0)
            for side, target in [('left', left), ('right', right)]:
                q1 = sim.state(f'{side}_dex1_finger_joint_1')[0]
                q2 = sim.state(f'{side}_dex1_finger_joint_2')[0]
                self.assertAlmostEqual(q1, q2, delta=5e-5)
                self.assertAlmostEqual(q1, target * GRIP_SCALE, delta=2e-4)
        self.assertFalse(any(w.number for w in sim.data.warning))

    def test_sparse_routing_crc_independent_command_validation_watchdog_and_disable(self):
        sim = self.sim
        command = self.command()
        command[26] = (1, .2, 0., 0., 80., 4.)
        self.assertTrue(sim.accept_command(command, now=0))
        sim.control(now=0)
        self.assertGreater(sim.data.ctrl[sim.joints['right_wrist_roll_joint'][0]], 10)
        self.assertLess(abs(sim.data.ctrl[sim.joints['left_wrist_roll_joint'][0]]), 1)
        invalid = command.copy()
        invalid[26] = (1, math.nan, 0., 0., 80., 4.)
        self.assertFalse(sim.accept_command(invalid, now=.1))
        sim.step(now=WATCHDOG + .01)
        self.assertTrue(sim.timed_out)
        self.assertIsNone(sim.active_command)
        disabled = {slot: (0, 0., 0., 0., 0., 0.) for slot in SLOTS}
        sim.accept_command(disabled, now=1)
        sim.control(now=1)
        for name in SLOTS.values():
            self.assertEqual(sim.data.ctrl[sim.joints[name][0]], 0)


if __name__ == '__main__':
    unittest.main()
