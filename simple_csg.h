#pragma once
#ifndef SIMPLE_CSG_H
#define SIMPLE_CSG_H


#include <cgv\math\fvec.h>
#include <cgv\media\mesh\simple_mesh.h>
#include "icosphere.h"
#include "ray_intersection.h"


enum class CSG_Intersect_Region {
	INSIDE = 1,
	OUTSIDE = -1,
	BOTH = 0
};

enum class CSG_Operation {
	CSG_UNION,
	CSG_DIFFERENCE,
	CSG_INTERSECTION
};

/// type of 3d vector
typedef typename cgv::math::fvec<float, 3> vec3;
/// type of simple mesh
typedef cgv::media::mesh::simple_mesh<float> mesh_type;

static class SimpleCSG {

public:
	/// Method to subtract two simple_meshes, the resulting mesh will be M1 without M2
	static mesh_type perform_csg_calculation(mesh_type& M1, HE_Mesh& HE, AabbTree<triangle>& aabb_tree, IcoSphere& IS, CSG_Operation op);

private:
	/// Method to add vertices to new Mesh with correct orientation
	static void create_face(mesh_type& M, int index1, int index2, int index3, bool orientation);
};


#endif //SIMPLE_CSG_H