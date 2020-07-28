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
	static mesh_type compute_intersections(mesh_type& M1, HE_Mesh& HE, AabbTree<triangle>& aabb_tree, IcoSphere& IS, CSG_Operation op);
	/// Method to subtract two simple_meshes, the resulting mesh will be M1 without M2
	static mesh_type Subtract(mesh_type& M1, IcoSphere& M2);

	/// Method to build new Mesh
	static mesh_type build_mesh(HE_Mesh& HE, std::map<HE_Edge*, CSG_Intersect_Region> edge_regions,
		std::map<HE_Edge*, vec3> intersect_points, CSG_Operation op);
};


#endif //SIMPLE_CSG_H