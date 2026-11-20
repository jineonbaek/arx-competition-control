from pathlib import Path
import subprocess,unittest
SOURCE=(Path(__file__).resolve().parents[1]/'start_navigation.sh').read_text()
class Launcher(unittest.TestCase):
 def run_case(self,mode,old_mode=''):
  case=SOURCE.split('case "$mode" in\n',1)[1].split('\nesac',1)[0]
  reuse='same=false\n'+SOURCE.split('same=false\n',1)[1].split('if ! $force_new',1)[0]
  prefix='''set -e
mode="$1"; old_mode="$2"; checkpoint=""; saved=""; resume=false; orient=false; competition=false; force_new=false; old="$3"
competition_target(){ echo /version/latest/room.yaml; }; session_idle(){ return 0; }; session_live(){ [[ -n "$old_mode" ]]; }; zenity(){ if [[ "$mode" == mapping ]]; then echo "继续比赛地图（默认）"; else echo "比赛地图（默认）"; fi; }; fail(){ exit 1; }; exec 9>/dev/null
'''
  import tempfile
  with tempfile.TemporaryDirectory() as tmp:
   (Path(tmp)/'competition-map').touch()
   out=subprocess.check_output(['bash','-c',prefix+'case "$mode" in\n'+case+'\nesac\n'+reuse+'\necho "$mode|$resume|$orient|$competition|${use_live_hint:-false}"','test',mode,old_mode,tmp],text=True).strip()
  return out
 def test_mapping_defaults_to_shared_editable_map(self):self.assertEqual(self.run_case('mapping'),'mapping|true|false|true|false')
 def test_navigation_defaults_to_shared_editable_map(self):self.assertEqual(self.run_case('navigation'),'mapping-navigation|true|false|true|false')
 def test_same_live_mode_does_not_resume_or_restart(self):self.assertEqual(self.run_case('mapping','mapping'),'mapping|false|false|true|true')
 def test_switch_uses_fresh_pose_hint(self):self.assertEqual(self.run_case('navigation','mapping'),'mapping-navigation|true|false|true|true')
 def test_new_direction_never_resumes_old_map(self):self.assertEqual(self.run_case('set-map-direction','mapping'),'mapping|false|true|true|false')
if __name__=='__main__':unittest.main()
