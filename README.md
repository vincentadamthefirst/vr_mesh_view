# VR Mesh View

Implementation of the mesh_view in VR. <br>
Environment can be toggled by using booleans in the ```vr_mesh_view() ```-constructor.
All inputs are handled in ```handle()``` individually, for each input a separate event gets fired! The workaround would
be to use the added ```VrController``` struct but this couldn't be done yet because the button events couldnt be tested
yet.

## Introduction :star:
1.  Project is based on the CGV-Framework develop branch <https://github.com/sgumhold/cgv>
2.  A working framework installation under windows is required
3.  See documentation for setup instructions <https://wwwpub.zih.tu-dresden.de/~gumhold/cgv/html/install.html>

## How to use our program :+1:
1. You should have VIVE controllers;
2. We have two modes: animation and editing mode;
3. Menu button on each controller can switch between two modes;
4. You can see useful information from a quad in the VR;
5. Mode introduction:
    - Mesh editing mode:
	    -Left Controller:
          - Stick up: Vertex Manipulation
          - Stick down: Select Face for Smoothing
          - Stick right: Apply Smoothing 
          - Stick left: Tessellation
      -Right Controller:
        -Stick up: Vertex Deletion
        -Stick down: 
        -Stick right: Recalculate Volume, Surface, shortest distance to mesh
        -Stick left: Recalculate Volume, Surface, shortest distance to mesh with acceleration data structure
     -Left Controller: Translation and Rotation of meshes
      -TODO
      -TODO
	   -Right Controller:
      -TODO
      -TODO

## Members of our team :two_men_holding_hands: :two_women_holding_hands:
Erdem Uenal, Boyang Liu, Christian Gärber, Vincent Adam, Ye Tao, Stefanie Krell

