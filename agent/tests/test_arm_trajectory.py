"""Exercise the driver interpolation without ROS, vendor SDK, CAN or motor access."""
import json
import subprocess
import tempfile
import unittest
from pathlib import Path
import numpy as np
from agent.grasp_path import TimedJointPath


class DriverTrajectoryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory()
        cls.binary=Path(cls.tmp.name)/'test_arm_trajectory'
        source=Path(__file__).resolve().parents[2]/'web/hardware/test_arm_trajectory.cpp'
        subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',str(source),
                        '-o',str(cls.binary)],check=True,capture_output=True,text=True,timeout=60)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_driver_rejects_unsafe_input_and_latches_interruptions(self):
        result=subprocess.run([str(self.binary)],check=True,capture_output=True,text=True,timeout=10)
        self.assertIn('arm trajectory checks passed',result.stdout)

    def test_driver_matches_python_checked_path_at_every_sample(self):
        q=np.radians([[0,20,40,10,0,-90],[4,17,42,11,0,-85],[2,21,43,9,0,-86]])
        path=TimedJointPath(q,.7,joint_speed_deg_s=[5.7]*6,joint_acceleration_deg_s2=[12.]*6,
                            tcp_speed_mm_s=20.,tcp_acceleration_mm_s2=40.)
        message=dict(version=1,request_id='parity',interpolation='segment_quintic_scalar',
                     time_from_start_s=path.times.tolist(),joint_waypoints_rad=q.tolist(),
                     test_policy=[5.7,12.,.5,2.,.05,.15,60.,.2])
        fixture=Path(self.tmp.name)/'fixture.json';fixture.write_text(json.dumps(message))
        result=subprocess.run([str(self.binary),str(fixture)],check=True,capture_output=True,text=True,timeout=10)
        samples=json.loads(result.stdout)
        expected=[path.sample(float(path.times[-1])*i/1000)[0] for i in range(1001)]
        np.testing.assert_allclose(samples,expected,atol=1e-12,rtol=0)

if __name__=='__main__':unittest.main()
