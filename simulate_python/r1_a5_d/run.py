#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path
import time

import mujoco
from simulation import Simulation


def main():
    parser = argparse.ArgumentParser(description='R1-A5-D + dual Dex1-1 MuJoCo simulation')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', type=float, default=0, help='simulated seconds; 0 runs until closed')
    parser.add_argument('--demo', action='store_true', help='offline wrist and gripper motion')
    parser.add_argument('--mobile', action='store_true', help='experimental free chassis with approximate contacts')
    parser.add_argument('--dds', action='store_true', help='HG DDS on loopback / domain 1')
    parser.add_argument('--render', type=Path, help='save final camera view (MUJOCO_GL=egl for headless)')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration < 0 or (args.headless and args.duration == 0):
        parser.error('headless requires --duration > 0')
    if args.demo and args.dds:
        parser.error('--demo and --dds are separate command sources')
    sim = Simulation(args.mobile)
    bridge = viewer = None
    try:
        if args.dds:
            from dds_bridge import Bridge
            bridge = Bridge()
            print('DDS simulation: interface=lo domain=1, HG sparse slots, mode_machine=10', flush=True)
        if not args.headless:
            import mujoco.viewer as mj_viewer
            viewer = mj_viewer.launch_passive(sim.model, sim.data)
            viewer.cam.lookat[:] = [0, 0, .85]
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 2.7, 135, -15
            viewer.opt.geomgroup[3] = 0
        steps = round(args.duration / sim.model.opt.timestep) if args.duration else None
        i = 0
        wall_start = time.monotonic()
        while (steps is None or i < steps) and (viewer is None or viewer.is_running()):
            start = time.monotonic()
            if bridge:
                bridge.update(sim)
            sim.step(args.demo)
            if bridge:
                bridge.publish(sim)
            if viewer and i % 10 == 0:
                viewer.sync()
            i += 1
            if viewer or bridge:
                time.sleep(max(0, sim.model.opt.timestep - (time.monotonic() - start)))
        physics_wall_time = time.monotonic() - wall_start
        if args.render:
            from PIL import Image
            args.render.parent.mkdir(parents=True, exist_ok=True)
            camera = mujoco.MjvCamera()
            mujoco.mjv_defaultCamera(camera)
            camera.lookat[:], camera.distance, camera.azimuth, camera.elevation = [0, 0, .85], 2.7, 135, -15
            option = mujoco.MjvOption()
            option.geomgroup[3] = 0
            with mujoco.Renderer(sim.model, height=720, width=960) as renderer:
                renderer.update_scene(sim.data, camera, scene_option=option)
                Image.fromarray(renderer.render()).save(args.render)
        report = sim.summary()
        report['physics_wall_time_s'] = physics_wall_time
        report['physics_steps_per_wall_second'] = i / report['physics_wall_time_s']
        if bridge:
            report['dds'] = {'received': bridge.received, 'rejected': bridge.rejected, 'watchdog_hold': sim.timed_out}
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2))
    finally:
        if viewer:
            viewer.close()
        if bridge:
            bridge.close()


if __name__ == '__main__':
    main()
