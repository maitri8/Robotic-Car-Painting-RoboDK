from robodk.robolink import *
from robodk.robomath import *

RDK = Robolink()
frame = RDK.Item('Car_Frame')
frame.setPose(transl(0, 2750, 0))
p = frame.Pose().Pos()
RDK.ShowMessage("Car_Frame is now at X=%.0f Y=%.0f Z=%.0f" % (p[0], p[1], p[2]), True)
