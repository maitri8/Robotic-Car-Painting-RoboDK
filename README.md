Robotic Car Painting Cell - RoboDK Simulation
Name: Maitri Maribasayyanavar
Software: RoboDK (trial) with Python API
Robot: ABB IRB 6640-130/3.2

HOW TO RUN
1. Open Car_Painting_Cell_v2.rdk in RoboDK.
2. Double-click the Python program "paint_all_sim" in the station tree.
   The robot paints the roof, hood and trunk with blue paint pieces.

FOLDERS
- Scripts/          Python scripts used to build, test and run the station.
- Station_Objects/  Exported station items (car, floor, walls, spray tool).

FINAL RESULT
92 blue paint pieces placed on the roof, hood and trunk, 0 stripes skipped.

LIMITATIONS
- The side of each panel farthest from the robot is not painted (outside
  reach from one base position).
- Side panels (doors, fenders) are not painted.
- The blue pieces show where the gun has passed. They are not a physical
  paint simulation.
