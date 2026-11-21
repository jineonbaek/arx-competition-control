import contextlib,io,json,sys,tempfile,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
import yaml
from PIL import Image
import room_match

class ObstacleROI(unittest.TestCase):
 def setUp(self):
  self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup);self.root=Path(self.tmp.name)
 def fixture(self,frame='7aa8506272a6422595877381d57483aa'):
  resolution=.10;origin=np.array([-15.,-6.]);image=np.full((120,180),254,dtype=np.uint8)
  image[30:40,30:50]=0 # More than25 legitimate landmarks inside screenshot ROI.
  image[115,75:125]=0 # Perfect, but outside the screenshot ROI.
  Image.fromarray(image[::-1]).save(self.root/'room.pgm')
  cfg=dict(image='room.pgm',resolution=resolution,origin=[float(origin[0]),float(origin[1]),0.],negate=0,occupied_thresh=.65,free_thresh=.196)
  (self.root/'room.yaml').write_text(yaml.safe_dump(cfg));(self.root/'coordinate-frame.json').write_text(json.dumps(dict(coordinate_id=frame)))
  truth=np.array([-5.,0.,0.]);laser=truth[:2]+[.18533,-.000003]
  endpoints=origin+np.column_stack((np.arange(75,125)+.5,np.full(50,115.5)))*resolution
  points=endpoints-laser
  (self.root/'hint.json').write_text(json.dumps(dict(frame='map',child_frame='base_link',x_m=truth[0],y_m=truth[1],yaw_rad=truth[2])))
  return image,resolution,origin,points
 def run_main(self,hint=True,frame='7aa8506272a6422595877381d57483aa'):
  _,_,_,points=self.fixture(frame);args=['room_match',str(self.root/'room.yaml'),str(self.root/'result.json')]
  if hint:args+=['--live-hint',str(self.root/'hint.json')]
  with patch.object(sys,'argv',args),patch.object(room_match,'collect',return_value=(points,points)),patch.object(room_match.rclpy,'init'),patch.object(room_match.rclpy,'shutdown'),contextlib.redirect_stdout(io.StringIO()):room_match.main()
  return json.loads((self.root/'result.json').read_text())
 def test_outside_only_returns_cannot_pass_initial_alignment(self):
  report=self.run_main();self.assertFalse(report['acceptable'],report)
  self.assertEqual(report['obstacle_bounds_m'],[-12.70,-4.10,1.44,3.98]);self.assertEqual(report['candidates'],[])
 def test_noncompetition_map_retains_original_hinted_matching(self):
  report=self.run_main(frame='different-map-frame');self.assertTrue(report['acceptable'],report);self.assertNotIn('obstacle_bounds_m',report)
 def test_global_match_also_rejects_outside_only_geometry(self):self.assertFalse(self.run_main(hint=False)['acceptable'])
 def test_outside_obstacles_do_not_change_distance_field(self):
  from localization_obstacles import ObstacleRegion
  region=ObstacleRegion((1.,1.,9.,9.),(12,12),1.,np.array([0.,0.]));occupied=region.mask.copy()
  base=region.distances(occupied,1.);changed=occupied ^ ~region.mask
  np.testing.assert_array_equal(base,region.distances(changed,1.))
  values=region.sample(base,np.array([[0,0],[4,4],[11,11],[-1,4]]));self.assertTrue(np.isnan(values[[0,2,3]]).all());self.assertEqual(values[1],0.)
 def test_insufficient_roi_returns_are_rejected_and_outside_hits_are_not_scored(self):
  from localization_obstacles import evidence,scores
  self.assertIsNone(evidence(np.array([.01]*24+[np.nan]*100),True));self.assertEqual(float(scores(np.array([.01]*24))),1.)
  result=evidence(np.array([.01]*25+[np.nan]*100),True);self.assertEqual(result['used_points'],25);self.assertEqual(result['matched_points'],25);self.assertAlmostEqual(result['score_m'],.01)
 def test_missing_map_coverage_and_insufficient_landmarks_are_rejected(self):
  from localization_obstacles import ObstacleRegion
  with self.assertRaises(ValueError):ObstacleRegion((-1.,0.,5.,5.),(6,6),1.,np.array([0.,0.]))
  region=ObstacleRegion((1.,1.,9.,9.),(12,12),1.,np.array([0.,0.]))
  with self.assertRaises(ValueError):region.distances(np.zeros((12,12),bool),1.)
 def test_small_matching_subset_does_not_beat_strong_evidence(self):
  from localization_obstacles import rank_by_region_evidence
  candidates=[dict(score_m=0.,matched_points=1,id='tiny'),dict(score_m=.04,matched_points=100,id='supported')]
  rank_by_region_evidence(candidates);self.assertEqual(min(candidates,key=lambda c:c['score_m'])['id'],'supported')
if __name__=='__main__':unittest.main()
