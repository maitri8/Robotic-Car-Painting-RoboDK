from robodk.robolink import *
from robodk.robomath import *
import numpy as np

RDK = Robolink()
robot = RDK.Item('ABB IRB 6640-130/3.2')
car = RDK.Item('Car_Body')
tool = RDK.Item('Paint_Spray_Gun')

# car bounding box in station coordinates
pts, _ = car.GetPoints(FEATURE_OBJECT_MESH)
p = np.array(pts)[:, :3]
T = np.array(car.Pose().rows)
w = (T[:3, :3] @ p.T).T + T[:3, 3]
mn, mx = w.min(axis=0), w.max(axis=0)
cy = (mn[1] + mx[1]) / 2
stand = 250  # gun distance from the surface, mm

def mk(pos, x, y, z):
    return Mat([[x[0], y[0], z[0], pos[0]],
                [x[1], y[1], z[1], pos[1]],
                [x[2], y[2], z[2], pos[2]],
                [0, 0, 0, 1]])

def reachable(pose):
    try:
        j = robot.SolveIK(pose, tool=tool.PoseTool())
        return len(j.list()) == 6
    except Exception:
        return False

xs = list(range(-2400, 2401, 400))
zs = 0.6 * mx[2]
tests = {
    'Top (gun pointing down)': [mk([x, cy, mx[2] + stand], [1,0,0], [0,-1,0], [0,0,-1]) for x in xs],
    'Near side (gun pointing +Y)': [mk([x, mn[1] - stand, zs], [1,0,0], [0,0,-1], [0,1,0]) for x in xs],
    'Far side (gun pointing -Y)': [mk([x, mx[1] + stand, zs], [1,0,0], [0,0,1], [0,-1,0]) for x in xs],
}

msg = "Car size: %.0f x %.0f x %.0f mm\n" % tuple(mx - mn)
for name, poses in tests.items():
    ok = [reachable(P) for P in poses]
    bad = [xs[i] for i, o in enumerate(ok) if not o]
    msg += "\n%s: %d of %d reachable" % (name, sum(ok), len(ok))
    if bad:
        msg += "\n   NOT reachable at X = %s" % bad
RDK.ShowMessage(msg, True)
