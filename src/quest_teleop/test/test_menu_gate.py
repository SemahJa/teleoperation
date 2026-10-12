import unittest
from quest_teleop.menu_recorder import MenuGate
from quest_teleop_interfaces.msg import QuestControllerState, QuestButtonState

class MenuGateTest(unittest.TestCase):
    def setUp(self): self.gate=MenuGate(); self.seq=0; self.now=10000000000
    def sample(self, pressed, **changes):
        self.seq+=1; self.now+=20000000
        m=QuestControllerState(); m.controller_id='left'; m.session_id='session'
        m.sequence=self.seq; m.source_time_pc_monotonic_ns=self.now
        m.source_time_valid=m.clock_sync_valid=m.connection_valid=m.session_focused=True
        m.reference_space='LOCAL'; m.reference_space_generation=1; m.clock_uncertainty_ns=1000000
        b=QuestButtonState(); b.name='menu'; b.active=True; b.pressed=pressed; m.buttons=[b]
        for name,value in changes.items(): setattr(m,name,value)
        return self.gate.update(m,self.now)
    def wait(self):
        for _ in range(20): self.sample(False)
    def test_release_edge_debounce_and_no_hold_repeat(self):
        self.assertFalse(self.sample(True)); self.assertFalse(self.sample(False))
        self.assertTrue(self.sample(True)); self.assertFalse(self.sample(True))
        self.sample(False); self.assertFalse(self.sample(True))
        self.wait(); self.assertTrue(self.sample(True))
    def test_session_order_timeout_and_invalid_input_require_release(self):
        self.sample(False); self.sample(True); self.wait()
        self.assertFalse(self.sample(True,session_id='new'))
        self.assertFalse(self.sample(True,session_id='new'))
        self.sample(False,session_id='new'); self.assertTrue(self.sample(True,session_id='new'))
        self.wait(); self.sample(False,clock_sync_valid=False)
        self.assertFalse(self.sample(True)); self.sample(False)
        self.gate.watchdog(self.now+200000000)
        self.assertFalse(self.sample(True))
        self.sample(False); self.wait()
        self.assertFalse(self.sample(True,sequence=0)); self.assertFalse(self.sample(True))
    def test_right_input_cannot_toggle_left_and_tracking_not_needed(self):
        self.assertFalse(self.sample(False,controller_id='right'))
        self.assertFalse(self.sample(True,controller_id='right'))
        self.sample(False)
        self.assertTrue(self.sample(True,position_tracked=False,pose_action_active=False))
        self.sample(False); self.wait()
        self.assertFalse(self.sample(True,clock_uncertainty_ns=11000000))
