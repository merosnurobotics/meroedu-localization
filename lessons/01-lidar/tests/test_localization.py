import unittest,math,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from wall_localizer import Pose,wall_range,score,grid_search
class LocalizationTest(unittest.TestCase):
    def test_cardinal_wall_distances(self):
        self.assertAlmostEqual(wall_range(.5,0,0),1.5)
        self.assertAlmostEqual(wall_range(.5,0,math.pi),2.5)
    def test_recovers_known_yaw(self):
        p=Pose(.63,-.47,.35)
        beams=[(i*math.tau/24,wall_range(p.x,p.y,p.yaw+i*math.tau/24)) for i in range(24)]
        found,_=grid_search(beams,p.yaw)
        self.assertLess(math.hypot(found.x-p.x,found.y-p.y),.01)
    def test_square_is_ambiguous(self):
        p=Pose(.6,-.4,.2);q=Pose(.4,.6,.2+math.pi/2)
        beams=[(i*math.tau/24,wall_range(p.x,p.y,p.yaw+i*math.tau/24)) for i in range(24)]
        self.assertLess(score(q,beams),1e-8)
    def test_insufficient_data_is_rejected(self):
        with self.assertRaises(ValueError):grid_search([(0,1)]*7,0)
if __name__=='__main__':unittest.main()
