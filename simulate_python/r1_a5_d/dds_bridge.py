"""HG LowCmd/LowState bridge for the local A5-D simulation only."""
from queue import Queue, Empty, Full
import time

import numpy as np
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.utils.crc import CRC
import unitree_sdk2py.core.channel as sdk_channel

from simulation import SLOTS, GRIP_SCALE

DOMAIN = 1
INTERFACE = 'lo'
MODE_MACHINE = 10


def initialize_dds():
    # This workspace's CycloneDDS 0.10.2 library aborts while printing the SDK's
    # config tracing mask (do_print_uint32_bitset). Use explicit local config
    # without tracing; keep the shared upstream SDK and C library unchanged.
    sdk_channel.ChannelConfigHasInterface = '''<CycloneDDS><Domain Id="any">
      <General><Interfaces><NetworkInterface name="$__IF_NAME__$" multicast="false"/></Interfaces>
      <AllowMulticast>false</AllowMulticast></General>
      <Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>20</MaxAutoParticipantIndex>
      <Peers><Peer Address="127.0.0.1"/></Peers></Discovery>
    </Domain></CycloneDDS>'''
    ChannelFactoryInitialize(DOMAIN, INTERFACE)


class Bridge:
    def __init__(self):
        initialize_dds()
        self.crc = CRC()
        self.received = self.rejected = 0
        self.pending = Queue(maxsize=1)
        self.pub = ChannelPublisher('rt/lowstate', LowState_)
        self.pub.Init()
        self.sub = ChannelSubscriber('rt/lowcmd', LowCmd_)
        self.sub.Init(self.receive, 1)

    def receive(self, msg):
        if msg.mode_machine != MODE_MACHINE or msg.mode_pr != 0 or msg.crc != self.crc.Crc(msg):
            self.rejected += 1
            return
        snapshot = {slot: (m.mode, m.q, m.dq, m.tau, m.kp, m.kd)
                    for slot in SLOTS for m in [msg.motor_cmd[slot]]}
        # DDS callback never touches mjData; physics thread owns all simulation writes.
        try:
            self.pending.get_nowait()
        except Empty:
            pass
        try:
            self.pending.put_nowait((snapshot, time.monotonic()))
        except Full:
            pass

    def update(self, sim):
        latest = None
        while True:
            try:
                latest = self.pending.get_nowait()
            except Empty:
                break
        if latest is not None:
            if sim.accept_command(latest[0], latest[1]):
                self.received += 1
            else:
                self.rejected += 1

    def publish(self, sim):
        state = unitree_hg_msg_dds__LowState_()
        state.mode_machine = MODE_MACHINE
        state.tick = int(sim.data.time * 1000) & 0xffffffff
        for slot, name in SLOTS.items():
            q, dq, tau = sim.state(name)
            scale = GRIP_SCALE if slot in (31, 33) else 1.
            if scale != 1:
                tau += sim.state(name[:-1] + '2')[2]
            motor = state.motor_state[slot]
            motor.mode, motor.q, motor.dq, motor.tau_est = 1, q / scale, dq / scale, tau * scale
        state.imu_state.quaternion = sim.data.sensor('imu_quat').data.tolist()
        state.imu_state.gyroscope = sim.data.sensor('imu_gyro').data.tolist()
        state.imu_state.accelerometer = sim.data.sensor('imu_acc').data.tolist()
        rpy = np.zeros(3)
        quat = state.imu_state.quaternion
        w, x, y, z = quat
        rpy[:] = [np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                  np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
                  np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))]
        state.imu_state.rpy = rpy.tolist()
        state.crc = self.crc.Crc(state)
        self.pub.Write(state)

    def close(self):
        self.sub.Close()
        self.pub.Close()
