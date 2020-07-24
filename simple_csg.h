#pragma once

#include <cgv\math\fvec.h>
#include <cgv\media\mesh\simple_mesh.h>

/// type of 3d vector
typedef typename cgv::math::fvec<float, 3> vec3;
/// type of simple mesh
typedef cgv::media::mesh::simple_mesh<float> mesh_type;

static class SimpleCSG {

public:
	/// Method to subtract two simple_meshes, the resulting mesh will be M1 without M2
	static mesh_type Subtract(mesh_type& M1, mesh_type& M2);
};