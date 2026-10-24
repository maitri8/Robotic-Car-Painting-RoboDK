from robodk.robolink import *
from robodk.robomath import *
import numpy as np

RDK = Robolink()
robot = RDK.Item('ABB IRB 6640-130/3.2')
car = RDK.Item('Car_Body')
tool = RDK.Item('Paint_Spray_Gun')
frame = RDK.Item('Car_Frame')

# delete old roof targets/program if the script is run again
for it in RDK.ItemList(ITEM_TYPE_TARGET):
    if it.Name().startswith('Roof_'):
        it.Delete()
old = RDK.Item('Paint_Roof')
if old.Valid():
    old.Delete()

# car mesh in station coordinates
pts, _ = car.GetPoints(FEATURE_OBJECT_MESH)
p = np.array(pts)[:, :3]
T = np.array(car.PoseAbs().rows)
w = (T[:3, :3] @ p.T).T + T[:3, 3]
top = w[:, 2].max()

# flat roof area = points within 60 mm of the highest point
roof = w[w[:, 2] > top - 60]
x0, x1 = roof[:, 0].min() + 50, roof[:, 0].max() - 50
y0, y1 = roof[:, 1].min() + 50, roof[:, 1].max() - 50

stand = 350      # gun distance from the surface, mm
tilt = 40        # degrees, leaning away from the robot
spacing = 200    # distance between stripes, mm
t = np.radians(tilt)
d = np.array([0, np.sin(t), -np.cos(t)])   # spray direction
xa = np.array([1.0, 0, 0])
ya = np.cross(d, xa)

def abs_pose(x, y):
    pos = np.array([x, y, top]) - stand * d
    return Mat([[xa[0], ya[0], d[0], pos[0]],
                [xa[1], ya[1], d[1], pos[1]],
                [xa[2], ya[2], d[2], pos[2]],
                [0, 0, 0, 1]])

def reachable(P):
    try:
        return len(robot.SolveIK(P, tool=tool.PoseTool()).list()) == 6
    except Exception:
        return False

prog = RDK.AddProgram('Paint_Roof', robot)
prog.setPoseFrame(frame)
prog.setPoseTool(tool)
inv_frame = frame.PoseAbs().inv()

ys = np.arange(y0, y1 + 1, spacing)
n_ok, n_bad, k = 0, 0, 0
for i, y in enumerate(ys):
    xs = [x0, x1] if i % 2 == 0 else [x1, x0]   # zig-zag
    for j, x in enumerate(xs):
        P = abs_pose(x, y)
        if not reachable(P):
            n_bad += 1
            continue
        k += 1
        tg = RDK.AddTarget('Roof_T%02d' % k, frame, robot)
        tg.setAsCartesianTarget()
        tg.setPose(inv_frame * P)
        if n_ok == 0 or j == 0:
            prog.MoveJ(tg)
        else:
            prog.MoveL(tg)
        n_ok += 1

RDK.ShowMessage("Roof: %.0f x %.0f mm, %d stripes\nTargets created: %d, unreachable (skipped): %d"
                % (x1 - x0, y1 - y0, len(ys), n_ok, n_bad), True)
