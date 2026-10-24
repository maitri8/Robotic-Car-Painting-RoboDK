from robodk.robolink import *

RDK = Robolink()
robot = RDK.Item('ABB IRB 6640-130/3.2')
tool = RDK.Item('Paint_Spray_Gun')
car = RDK.Item('Car_Body')
frame = RDK.Item('Car_Frame')
robot.setPoseFrame(frame)
robot.setPoseTool(tool)

RDK.Spray_Clear()
RDK.Spray_Add(tool, car)
RDK.Spray_SetState(SPRAY_OFF)

targets = sorted([t for t in RDK.ItemList(ITEM_TYPE_TARGET) if t.Name().startswith('DeckB_T')],
                 key=lambda t: t.Name())
robot.MoveJ(targets[0])
RDK.Spray_SetState(SPRAY_ON)
for t in targets[1:4]:
    robot.MoveL(t)
RDK.Spray_SetState(SPRAY_OFF)
RDK.ShowMessage('Test done', True)
