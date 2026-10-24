from robodk.robolink import *
from robodk.robomath import *
import numpy as np

RDK = Robolink()
robot = RDK.Item('ABB IRB 6640-130/3.2')
tool = RDK.Item('Paint_Spray_Gun')
frame = RDK.Item('Car_Frame')

# park the arm away from the car so the height probe hits only the car
robot.setJoints([0, 0, 0, 0, 0, 0])

# delete old deck targets and programs
for it in RDK.ItemList(ITEM_TYPE_TARGET):
    if it.Name().startswith('DeckA_') or it.Name().startswith('DeckB_'):
        it.Delete()
for n in ['Paint_Deck_A', 'Paint_Deck_B']:
    old = RDK.Item(n)
    if old.Valid():
        old.Delete()

cy = 2755
stand, tilt = 350, 40
t = np.radians(tilt)
d = np.array([0, np.sin(t), -np.cos(t)])
xa = np.array([1.0, 0, 0])
ya = np.cross(d, xa)

def surf(x, y):
    hit, item, xyz = RDK.Collision_Line([x, y, 3000], [x, y, 0])
    if hit and item.Name() == 'Car_Body':
        return xyz[2]
    return None

def abs_pose(x, y, z):
    pos = np.array([x, y, z]) - stand * d
    return Mat([[xa[0], ya[0], d[0], pos[0]],
                [xa[1], ya[1], d[1], pos[1]],
                [xa[2], ya[2], d[2], pos[2]],
                [0, 0, 0, 1]])

def reachable(P):
    try:
        return len(robot.SolveIK(P, tool=tool.PoseTool()).list()) == 6
    except Exception:
        return False

inv_frame = frame.PoseAbs().inv()
zones = [('A', 1400, 2100), ('B', -2100, -1400)]
ys = np.linspace(cy - 600, cy + 600, 7)
msg = ''
for tag, xlo, xhi in zones:
    prog = RDK.AddProgram('Paint_Deck_' + tag, robot)
    prog.setPoseFrame(frame)
    prog.setPoseTool(tool)
    k, bad, zmin, zmax = 0, 0, 9999, 0
    for i, y in enumerate(ys):
        xs = [xlo, (xlo + xhi) / 2, xhi]
        if i % 2 == 1:
            xs = xs[::-1]
        first = True
        for x in xs:
            z = surf(x, y)
            if z is None:
                bad += 1
                continue
            P = abs_pose(x, y, z)
            if not reachable(P):
                bad += 1
                continue
            k += 1
            zmin, zmax = min(zmin, z), max(zmax, z)
            tg = RDK.AddTarget('Deck%s_T%02d' % (tag, k), frame, robot)
            tg.setAsCartesianTarget()
            tg.setPose(inv_frame * P)
            tg.setVisible(False)
            if first:
                prog.MoveJ(tg)
                first = False
            else:
                prog.MoveL(tg)
    msg += "End %s: X %d to %d\n   surface Z %.0f to %.0f\n   targets %d, skipped %d\n" % (
        tag, xlo, xhi, zmin, zmax, k, bad)

RDK.ShowMessage(msg, True)
