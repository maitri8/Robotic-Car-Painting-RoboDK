from robodk.robolink import *
from robodk.robomath import *
import numpy as np

RDK = Robolink()
RDK.setCollisionActive(COLLISION_ON)
robot = RDK.Item('ABB IRB 6640-130/3.2')
tool = RDK.Item('Paint_Spray_Gun')
home = [0, 0, 0, 0, 0, 0]
robot.setJoints(home)   # park arm so the height probe only hits the car

cy, stand = 2755, 350

def surf(x, y):
    hit, item, xyz = RDK.Collision_Line([x, y, 3000], [x, y, 0])
    if hit and item.Name() == 'Car_Body':
        return xyz[2]
    return None

def pose(x, y, z, a, b):
    a, b = np.radians(a), np.radians(b)
    d = np.array([np.sin(a), np.sin(b)*np.cos(a), -np.cos(a)*np.cos(b)])
    ya = np.cross(d, [1.0, 0, 0]); ya /= np.linalg.norm(ya)
    xa = np.cross(ya, d)
    p = np.array([x, y, z]) - stand * d
    return Mat([[xa[0], ya[0], d[0], p[0]],
                [xa[1], ya[1], d[1], p[1]],
                [xa[2], ya[2], d[2], p[2]],
                [0, 0, 0, 1]])

def status(P):
    # returns 0 = unreachable, 1 = reachable but every solution collides, 2 = reachable and collision-free
    try:
        sols = robot.SolveIK_All(P, tool.PoseTool())
        cols = sols.Cols()
    except Exception:
        return 0
    cols = [c for c in cols if len(c) >= 6]
    if not cols:
        return 0
    for c in cols:
        robot.setJoints(c[:6])
        if RDK.Collisions() == 0:
            return 2
    return 1

zones = [('A', 1400, 2100), ('B', -2100, -1400)]
ys = np.linspace(cy - 600, cy + 600, 7)
tilts = [(0, 40), (0, -40), (0, 0), (40, 0), (-40, 0), (30, 30), (-30, 30)]
msg = ''
for tag, xlo, xhi in zones:
    msg += 'End %s (of 21 points)\n' % tag
    for (a, b) in tilts:
        c = [0, 0, 0]
        for y in ys:
            for x in [xlo, (xlo + xhi) / 2, xhi]:
                z = surf(x, y)
                if z is None:
                    continue
                c[status(pose(x, y, z, a, b))] += 1
        msg += '  tilt(%d,%d): OK %d | collide %d | unreachable %d\n' % (a, b, c[2], c[1], c[0])
robot.setJoints(home)
RDK.ShowMessage(msg, True)
