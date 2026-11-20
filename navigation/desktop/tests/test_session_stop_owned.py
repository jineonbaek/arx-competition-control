"""Inert shell functions prove grouped recovery sessions can be closed by ownership."""
import os,tempfile,subprocess,unittest
from pathlib import Path
SESSION=Path(os.environ.get('SESSION_SH',str(Path(__file__).resolve().parents[1]/'session.sh')))
class OwnedSessionStop(unittest.TestCase):
 def run_case(self,standard,owned):
  with tempfile.TemporaryDirectory() as d:
   Path(d,'launch.pid').write_text('123456\n')
   script='''source "$1"
session_live(){ return 0; }
session_idle(){ return 0; }
session_mode(){ echo navigation; }
stop_owned(){ return STANDARD; }
python3(){
 if [[ "$1" != desktop/session_processes.py ]];then return 99;fi
 if [[ "$2" == live ]];then echo live;return OWNED;fi
 if [[ "$2" == stop ]];then echo stop;return 0;fi
 return 98
}
stop_session "$2" skip_save
'''.replace('STANDARD',str(standard)).replace('OWNED',str(owned))
   return subprocess.run(['bash','-c',script,'inert',str(SESSION),d],text=True,capture_output=True)
 def test_verified_recovery_launch_stops_exact_owned_session(self):
  result=self.run_case(1,0);self.assertEqual(result.returncode,0);self.assertEqual(result.stdout.splitlines(),['live','stop'])
 def test_unowned_process_is_rejected(self):
  result=self.run_case(1,1);self.assertNotEqual(result.returncode,0);self.assertNotIn('stop',result.stdout.splitlines())
 def test_standard_launch_retains_existing_close_path(self):
  result=self.run_case(0,1);self.assertEqual(result.returncode,0);self.assertEqual(result.stdout.splitlines(),['stop'])
if __name__=='__main__':unittest.main()
