from robodk.robolink import *

RDK = Robolink()

def box(x0, x1, y0, y1, z0, z1):
    """12 triangles of an axis-aligned box, as a flat list of vertices."""
    v = [[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0],
         [x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]]
    faces = [(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
             (1,2,6),(1,6,5),(2,3,7),(2,7,6),(3,0,4),(3,4,7)]
    return [v[i] for f in faces for i in f]

# remove old objects if the script is run again
for name in ['Floor', 'Wall_Back', 'Wall_Left', 'Wall_Right']:
    old = RDK.Item(name)
    if old.Valid():
        old.Delete()

# 12 m x 8 m floor, top surface at Z = 0, robot base at (0,0,0)
floor = RDK.AddShape(box(-6000, 6000, -2500, 5500, -20, 0))
floor.setName('Floor')
floor.setColor([0.55, 0.55, 0.55, 1.0])

# 3 m high translucent walls: back and both ends (front left open for the camera)
wall_color = [0.6, 0.75, 0.9, 0.3]
w1 = RDK.AddShape(box(-6000, 6000, 5400, 5500, 0, 3000))
w1.setName('Wall_Back');  w1.setColor(wall_color)
w2 = RDK.AddShape(box(-6000, -5900, -2500, 5500, 0, 3000))
w2.setName('Wall_Left');  w2.setColor(wall_color)
w3 = RDK.AddShape(box(5900, 6000, -2500, 5500, 0, 3000))
w3.setName('Wall_Right'); w3.setColor(wall_color)

RDK.ShowMessage('Cell built: floor 12000 x 8000 mm, walls 3000 mm high', True)
