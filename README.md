# VR Mesh View

Implementation of the mesh_view in VR. <br>
Environment can be toggled by using booleans in the ```vr_mesh_view() ```-constructor.
All inputs are handled in ```handle()``` individually, for each input a separate event gets fired! The workaround would
be to use the added ```VrController``` struct but this couldn't be done yet because the button events couldnt be tested
yet.