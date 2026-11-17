from pathlib import Path
from queue import Queue
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dds_bridge import Bridge, CRC
from simulation import SLOTS
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_


class BridgeTests(unittest.TestCase):
    def test_reject_bad_crc_and_wrong_machine_and_snapshot_slots(self):
        bridge = Bridge.__new__(Bridge)
        bridge.crc = CRC()
        bridge.pending = Queue(maxsize=1)
        bridge.rejected = 0
        msg = unitree_hg_msg_dds__LowCmd_()
        msg.mode_machine = 10
        msg.crc = bridge.crc.Crc(msg)
        bridge.receive(msg)
        snapshot, _ = bridge.pending.get_nowait()
        self.assertEqual(set(snapshot), set(SLOTS))
        msg.motor_cmd[26].q = .25
        bridge.receive(msg)
        self.assertTrue(bridge.pending.empty())
        msg.crc = bridge.crc.Crc(msg)
        msg.mode_machine = 1
        bridge.receive(msg)
        self.assertTrue(bridge.pending.empty())
        self.assertEqual(bridge.rejected, 2)
        msg.mode_machine = 10
        msg.crc = bridge.crc.Crc(msg)
        for _ in range(100):
            bridge.receive(msg)
        self.assertEqual(bridge.pending.qsize(), 1)
        self.assertEqual(bridge.pending.get_nowait()[0][26][1], .25)


if __name__ == '__main__':
    unittest.main()
