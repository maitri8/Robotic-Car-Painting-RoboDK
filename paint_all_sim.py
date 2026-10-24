from robodk.robolink import *
from robodk.robomath import *
import numpy as np

RDK = Robolink()
robot = RDK.Item('ABB IRB 6640-130/3.2')
tool = RDK.Item('Paint_Spray_Gun')
car = RDK.Item('Car_Body')
frame = RDK.Item('Car_Frame')

frame.setPose(transl(0, 2750, 0))

for it in RDK.ItemList(ITEM_TYPE_OBJECT):
    if it.Name().startswith('PaintStrip'):
        it.Delete()

robot.setJoints([0, 0, 0, 0, 0, 0])   # park the arm so the height probe hits only the car

def surf(x, y):
    hit, item, xyz = RDK.Collision_Line([x, y, 3000], [x, y, 0])
    if hit and item.Name() == 'Car_Body':
        return xyz[2]
    return None

def seg_tris(x0, x1, y, half=100):
    za, zb = surf(x0, y), surf(x1, y)
    if za is None or zb is None:
        return []
    a = [x0, y - half, za + 3]
    b = [x1, y - half, zb + 3]
    c = [x1, y + half, zb + 3]
    d = [x0, y + half, za + 3]
    return [a, b, c, a, c, d]

N = 4                                   # pieces of paint per stripe segment
dy = 350 * np.sin(np.radians(40))

# step 1: build the plan (poses along each stripe + the paint piece under each)
plan = []
for prefix in ['Roof_T', 'DeckA_T', 'DeckB_T']:
    targets = sorted([t for t in RDK.ItemList(ITEM_TYPE_TARGET) if t.Name().startswith(prefix)],
                     key=lambda t: t.Name())
    ys = [t.Pose().Pos()[1] for t in targets]
    i = 0
    while i < len(targets):
        j = i + 1
        while j < len(targets) and abs(ys[j] - ys[i]) < 1:
            j += 1
        if j - i >= 2:
            steps = []
            for k in range(i, j - 1):
                ta, tb = targets[k], targets[k + 1]
                Pa, Pb = ta.Pose(), tb.Pose()
                va = np.array(Pa.Pos())
                vb = np.array(Pb.Pos())
                aa = np.array(ta.PoseAbs().Pos())
                ab = np.array(tb.PoseAbs().Pos())
                for s in range(N):
                    f0, f1 = s / N, (s + 1) / N
                    d1 = (vb - va) * f1
                    pose = transl(d1[0], d1[1], d1[2]) * Pa
                    x0 = aa[0] + (ab[0] - aa[0]) * f0
                    x1 = aa[0] + (ab[0] - aa[0]) * f1
                    steps.append((pose, seg_tris(x0, x1, aa[1] + dy)))
            plan.append((targets[i], steps))
        i = j

# step 2: paint
robot.setPoseFrame(frame)
robot.setPoseTool(tool)
RDK.Spray_Clear()
RDK.Spray_Add(tool, car)
RDK.Spray_SetState(SPRAY_OFF)

n, skipped = 0, 0
for start, steps in plan:
    try:
        robot.MoveJ(start)
    except Exception:
        skipped += 1
        continue
    RDK.Spray_SetState(SPRAY_ON)
    for pose, tris in steps:
        try:
            robot.MoveL(pose)
        except Exception:
            break
        if tris:
            n += 1
            s = RDK.AddShape(tris)
            s.setName('PaintStrip%03d' % n)
            s.setColor([0.1, 0.3, 0.9, 1.0])   # blue paint
    RDK.Spray_SetState(SPRAY_OFF)

RDK.ShowMessage('Painting finished. Blue paint pieces: %d, stripes skipped: %d' % (n, skipped), True)
