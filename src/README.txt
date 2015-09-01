Uber (inter)View - PCD viewing software
Author: Sam Musso

Build:
  mkdir ../bin; cd ../bin/
  cmake -DCMAKE_BUILD_TYPE=Release ../src/
  make ub-view

Examples:
  ./ub-view -h
  ./ub-view -f ../model.pcd

Usage Info:
  Load PCD File:
   This program takes a single .pcd filename at startup.
   Use -f flag.
  Save PCD snapshot:
    'z' Save the current view to snapshot file in `pwd`.
  Movement:
   This program provides translational movement along the cameras local xyz.
   [ Move left.
   \ Move right.
   ] Move forward.
   Apostra Move backward.
   Period Move down.
   / Move up.
  View:
   Click and drag to rotate the camera around the viewpoint.
   Use p,w,s keys for point, wire, and solid representations respectively.
   Wire & Solid views require mesh to be visible. See 'Meshing' below. 
  Color:
   Use the trackbars to change cloud coloring options.
  Meshing:
   A geometry mesh is generated from the point cloud data on program start.
   To reveal the mesh, choose an rgb value then press the 'm'.

Thanks for using ub-view.
