from robodk.robolink import *
import numpy as np

RDK = Robolink()
car = RDK.Item('Car_Body')
pts, _ = car.GetPoints(FEATURE_OBJECT_MESH)
a = np.array(pts)
T = np.array(car.PoseAbs().rows)
w = (T[:3, :3] @ a[:, :3].T).T + T[:3, 3]

cy = (w[:, 1].min() + w[:, 1].max()) / 2

def zr(x0, x1):
    inx = (w[:, 0] > x0) & (w[:, 0] < x1)
    near = abs(w[:, 1] - cy) < 700
    s = w[inx & near]
    if len(s) == 0:
        return "no points"
    return "%d pts, Z %.0f to %.0f" % (len(s), s[:, 2].min(), s[:, 2].max())

msg = "Total points: %d\n" % len(a)
for x0 in range(-2400, 2400, 400):
    msg += "X %5d..%5d: %s\n" % (x0, x0 + 400, zr(x0, x0 + 400))
RDK.ShowMessage(msg, True)
