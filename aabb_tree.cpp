#include "aabb_tree.h"
#include <cgv/media/mesh/simple_mesh.h>
#include <iostream>

typedef cgv::math::fvec<float, 3> vec3;
typedef cgv::media::mesh::simple_mesh<float> mesh_type;

void build_aabbtree_from_triangles(HE_Mesh * he, AabbTree<triangle>& tree) {
	std::cout << "Building aabb tree ..." << std::endl;
	tree.clear();
	std::vector<HE_Face*>* face_ = he->GetFaces();
	std::cout << "Numbers of faces:" << face_->size() << std::endl;
	for (auto face : *face_) {

		vec3 v_0 = he->GetVerticesForFace(face).at(0)->position;
		vec3 v_1 = he->GetVerticesForFace(face).at(1)->position;
		vec3 v_2 = he->GetVerticesForFace(face).at(2)->position;
		tree.insert(triangle(v_0, v_1, v_2));
	}

	tree.pre_build();

	std::cout << "Done." << std::endl;
}