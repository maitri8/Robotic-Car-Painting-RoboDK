from robodk.robolink import *
from robodk.robomath import *
import numpy as np

RDK = Robolink()
robot = RDK.Item('ABB IRB 6640-130/3.2')
car = RDK.Item('Car_Body')
tool = RDK.Item('Paint_Spray_Gun')

pts, _ = car.GetPoints(FEATURE_OBJECT_MESH)
p = np.array(pts)[:, :3]
T = np.array(car.PoseAbs().rows)
w = (T[:3, :3] @ p.T).T + T[:3, 3]
mn, mx = w.min(axis=0), w.max(axis=0)
cy = (mn[1] + mx[1]) / 2
top = mx[2]
stand = 250  # mm

def pose_for(P, tilt_deg):
    t = np.radians(tilt_deg)
    s, c = np.sin(t), np.cos(t)
    d = np.array([0, s, -c])      # spray direction: down, leaning away from the robot (+Y)
    x = np.array([1, 0, 0])
    y = np.cross(d, x)
    pos = np.array(P) - stand * d
    return Mat([[x[0], y[0], d[0], pos[0]],
                [x[1], y[1], d[1], pos[1]],
                [x[2], y[2], d[2], pos[2]],
                [0, 0, 0, 1]])

def reachable(pose):
    try:
        j = robot.SolveIK(pose, tool=tool.PoseTool())
        return len(j.list()) == 6
    except Exception:
        return False

xs = list(range(-2000, 2001, 400))
msg = "Roof-height points, car Y = %.0f mm (13... %d points per row)\n" % (cy, len(xs))
for tilt in [0, 20, 40]:
    msg += "\nTilt %d deg:" % tilt
    for dy in [-600, -300, 0]:
        n = sum(reachable(pose_for([x, cy + dy, top], tilt)) for x in xs)
        msg += "\n   %4d mm from centreline: %d of %d" % (dy, n, len(xs))
RDK.ShowMessage(msg, True)
