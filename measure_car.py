from robodk.robolink import *
from robodk.robomath import *
import numpy as np

RDK = Robolink()
car = RDK.Item('Car_Body')

# 1) scale uniformly so the long side (local Y) is 4800 mm
pts, _ = car.GetPoints(FEATURE_OBJECT_MESH)
p = np.array(pts)[:, :3]
length = p[:, 1].max() - p[:, 1].min()
car.Scale(4800.0 / length)

# 2) re-read the scaled mesh and rotate 90 deg about Z (x,y) -> (-y,x)
pts, _ = car.GetPoints(FEATURE_OBJECT_MESH)
p = np.array(pts)[:, :3]
rx, ry, rz = -p[:, 1], p[:, 0], p[:, 2]

# 3) centre the car in X, put its centreline 2300 mm from the robot base, wheels on Z=0
tx = -(rx.min() + rx.max()) / 2
ty = 2300 - (ry.min() + ry.max()) / 2
tz = -rz.min()
car.setPose(transl(tx, ty, tz) * rotz(pi / 2))

# 4) light grey colour (R,G,B,A from 0 to 1)
car.setColor([0.8, 0.8, 0.8, 1.0])

RDK.ShowMessage("Car size X,Y,Z = %.0f x %.0f x %.0f mm\nPlaced at X=%.0f Y=%.0f Z=%.0f" %
                (rx.max() - rx.min(), ry.max() - ry.min(), rz.max() - rz.min(), tx, ty, tz), True)
