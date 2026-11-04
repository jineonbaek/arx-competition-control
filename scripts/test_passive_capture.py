"""Offline command-boundary tests; no physical serial devices are opened."""
import unittest
from unittest.mock import patch
from passive_can_capture import adapter_command
class FakePort:
    def __init__(self,response=b'\r'):self.writes=[];self.response=response
    def write(self,data):self.writes.append(data)
    def flush(self):pass
    def read(self,size):return self.response
class CommandBoundaryTests(unittest.TestCase):
    @patch('passive_can_capture.time.sleep')
    def test_only_adapter_close_speed_and_listen_commands(self,sleep):
        p=FakePort();events=[]
        for command in [b'C\r',b'S8\r',b'L\r']:adapter_command(p,command,events)
        self.assertEqual(p.writes,[b'C\r',b'S8\r',b'L\r'])
    def test_can_transmit_normal_open_and_enable_are_rejected_before_write(self):
        for command in [b'O\r',b't0018FFFFFFFFFFFFFFFC\r',b'T0000000180000000000000000\r',b'r0010\r',b'R000000010\r',b'C\rO\r']:
            p=FakePort()
            with self.assertRaises(ValueError):adapter_command(p,command,[])
            self.assertEqual(p.writes,[])
    @patch('passive_can_capture.time.sleep')
    def test_adapter_error_does_not_trigger_normal_open(self,sleep):
        p=FakePort(b'\x07')
        with self.assertRaises(RuntimeError):adapter_command(p,b'L\r',[])
        self.assertEqual(p.writes,[b'L\r'])
if __name__=='__main__':unittest.main()
