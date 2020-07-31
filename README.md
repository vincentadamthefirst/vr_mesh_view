# VR Mesh View

## Introduction
Our project is based on the CGV framework. We have created an environment in the VR where you can load a mesh and manipulate it. With our program, you can **translate and rotate** your mesh with controller in the VR. You can also define a path and do the **animation**. Mesh editing is also possible, like **smoothing, tessellation, measuring and vertex manipulation**. We also provide the **constructive solid geometry operation(union, substraction, intersection)**. 

## How to start :star:
1.  Project is based on the CGV-Framework develop branch <https://github.com/sgumhold/cgv>;
2.  A working framework installation under windows is required;
3.  See documentation for setup instructions <https://wwwpub.zih.tu-dresden.de/~gumhold/cgv/html/install.html>.

## How to use our program :+1:
1. You should have VIVE controllers;
2. We have two modes: **animation** and **editing mode**;
3. ```Menu``` button on each controller can switch between two modes;
4. You can see useful information from a quad in the VR;
5. Mode introduction:
	- Mesh editing mode:
		- Left Controller:
			- ```Stick up```: Vertex Manipulation
			- ```Stick down```: Select Face for Smoothing
			- ```Stick right```: Apply Smoothing 
			- ```Stick left```: Tessellation
		- Right Controller:
			- ```Stick up```: Vertex Deletion
			- ```Stick down```: CSG Operation
			- ```Stick right```: Recalculate Volume, Surface
			- ```Stick left```: Shortest distance to mesh with acceleration data structure
			
	- Mesh animation mode:
		- Left Controller: Rotation of mesh
			- ```Stick down```: Define Path
		- Right Controller: Translation of mesh
      			- ```Stick ```: Animation

## Members of our team :two_men_holding_hands: :two_women_holding_hands:
Erdem Uenal, Boyang Liu, Christian Gärber, Vincent Adam, Ye Tao, Stefanie Krell

