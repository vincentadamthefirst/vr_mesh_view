#include "simple_csg.h"


void SimpleCSG::create_face(mesh_type& M, int index1, int index2, int index3, bool orientation) {

	M.start_face();
	if (orientation) {
		M.new_corner(index1);
		M.new_corner(index2);
		M.new_corner(index3);
	}
	else {
		M.new_corner(index1);
		M.new_corner(index3);
		M.new_corner(index2);
	}
}


mesh_type SimpleCSG::perform_csg_calculation(mesh_type& M1, HE_Mesh& HE, AabbTree<triangle>& aabb_tree, IcoSphere& IS, CSG_Operation op) {
	vec3 icosphere_center = IS.GetSphereCenter();
	float radius = IS.GetSphereRadius();

	std::map<vec3*, bool> meshPointInIcosphere;
	std::map<HE_Edge*, CSG_Intersect_Region> edge_regions;
	std::map<HE_Edge*, vec3> intersect_points;

	std::map<unsigned, bool> icoPointInMesh;

	std::vector<HE_Edge*> holeBoundaryEdgesMesh;
	std::map<int, int> holeBoundaryEdgesSphere;

	for (HE_Vertex* v : *HE.GetVertices()) {
		vec3* p = &(v->position);
		meshPointInIcosphere.insert(std::make_pair(p, (icosphere_center - *p).length() < radius));
	}

	for (HE_Edge* e : *HE.GetHalfEdges()) {
		vec3* p1 = &(e->origin->position);
		vec3* p2 = &(e->next->origin->position);

		if (meshPointInIcosphere.at(p1) && meshPointInIcosphere.at(p2)) {
			edge_regions.insert(std::make_pair(e, CSG_Intersect_Region::INSIDE));

		} else if (!meshPointInIcosphere.at(p1) && meshPointInIcosphere.at(p2)) {
			vec3 v1 = *p1 - icosphere_center;
			vec3 v2 = *p2 - icosphere_center;

			float d1 = v1.length();
			float d2 = v2.length();

			vec3 p_intersect = *p1 + ((radius - d1)/d2-d1) * (*p2 - *p1);

			edge_regions.insert(std::make_pair(e, CSG_Intersect_Region::BOTH));
			intersect_points.insert(std::make_pair(e, p_intersect));

			if (e->twin) {
				edge_regions.insert(std::make_pair(e->twin, CSG_Intersect_Region::BOTH));
				intersect_points.insert(std::make_pair(e->twin, p_intersect));
			}

		} else if (!meshPointInIcosphere.at(p1) && !meshPointInIcosphere.at(p2)) {
			edge_regions.insert(std::make_pair(e, CSG_Intersect_Region::OUTSIDE));
		}
	}

	bool centeroutside = false;

	for (int x = -1; x <= 1; x += 2) {
		for (int y = -1; y <= 1; y += 2) {
			for (int z = -1; z <= 1; z += 2) {
				ray_intersection::ray ray = ray_intersection::ray(icosphere_center, vec3(x,y,z));
				float t = 0.0;
				centeroutside = centeroutside || !ray_intersection::rayTreeIntersect(ray, aabb_tree, t);
			}
		}
	}

	for (auto i = 0; i < IS.GetVertices().size(); i++) {
		ray_intersection::ray ray = ray_intersection::ray(icosphere_center, IS.GetVertices()[i] - icosphere_center);
		float t = 0.0;
		bool intersect = ray_intersection::rayTreeIntersect(ray, aabb_tree, t);

		if (centeroutside == !intersect) {
			icoPointInMesh.insert(std::make_pair(i, false));
		}
		else {
			icoPointInMesh.insert(std::make_pair(i, centeroutside == (t < 1.0f)));
		}
	}

	std::vector<IcoSphere::TriangleIndices> ico_remaining;

	bool ico_in_mesh_remain = (op == CSG_Operation::CSG_DIFFERENCE || op == CSG_Operation::CSG_INTERSECTION);
	bool ico_out_mesh_remain = (op == CSG_Operation::CSG_UNION);
	bool mesh_in_ico_remain = (op == CSG_Operation::CSG_INTERSECTION);
	bool mesh_out_ico_remain = (op == CSG_Operation::CSG_DIFFERENCE || op == CSG_Operation::CSG_UNION);

	for (IcoSphere::TriangleIndices t : IS.GetTriangles()) { 
		bool v1_inside = icoPointInMesh.at(t.v1);
		bool v2_inside = icoPointInMesh.at(t.v2);
		bool v3_inside = icoPointInMesh.at(t.v3);

		if (v1_inside && v2_inside && v3_inside) {
			if (ico_in_mesh_remain) {
				ico_remaining.push_back(t);
			}
		}
		else if (!v1_inside && !v2_inside && !v3_inside) {
			if (ico_out_mesh_remain) {
				ico_remaining.push_back(t);
			}
		}
		else if (ico_in_mesh_remain) {
			if (v1_inside && v2_inside) {
				holeBoundaryEdgesSphere.insert(std::make_pair(t.v1, t.v2));
			}
			else if (v2_inside && v3_inside) {
				holeBoundaryEdgesSphere.insert(std::make_pair(t.v2, t.v3));
			}
			else if (v3_inside && v1_inside) {
				holeBoundaryEdgesSphere.insert(std::make_pair(t.v3, t.v1));
			}
		}
		else if (ico_out_mesh_remain) {
			if (!v1_inside && !v2_inside) {
				holeBoundaryEdgesSphere.insert(std::make_pair(t.v1, t.v2));
			}
			else if (!v2_inside && !v3_inside) {
				holeBoundaryEdgesSphere.insert(std::make_pair(t.v2, t.v3));
			}
			else if (!v3_inside && !v1_inside) {
				holeBoundaryEdgesSphere.insert(std::make_pair(t.v3, t.v1));
			}
		}
	}

	std::vector<HE_Face*> mesh_remaining;

	for (HE_Face* f : *HE.GetFaces()) {
		HE_Edge* originalEdge = f->adjacent;
		HE_Edge* edge = originalEdge;

		bool all_outside = true;
		bool all_inside = true;

		auto count = 0;
		HE_Edge* boundary; 

		do {
			all_inside = all_inside && edge_regions.at(edge) == CSG_Intersect_Region::INSIDE;
			all_outside = all_outside && edge_regions.at(edge) == CSG_Intersect_Region::OUTSIDE;

			if ((mesh_out_ico_remain && edge_regions.at(edge) == CSG_Intersect_Region::OUTSIDE) || 
				(mesh_in_ico_remain && edge_regions.at(edge) == CSG_Intersect_Region::INSIDE)) {
				count++;
				boundary = edge;
			}

			edge = edge->next;
		} while (edge != originalEdge);

		if (count == 1) {
			holeBoundaryEdgesMesh.push_back(boundary);
		}

		if (mesh_in_ico_remain && all_inside || mesh_out_ico_remain && all_outside) {
			mesh_remaining.push_back(f);
		}
	}

	if (ico_remaining.size() < 1 || mesh_remaining.size() < 1) {
		return M1;
	}

	mesh_type R;

	std::map<int, int> ico_vertex_index_mapping;
	std::map<int, int> mesh_vertex_index_mapping;

	std::map<int, std::vector<int>> holeCloseMapping;

	if (holeBoundaryEdgesSphere.size() == 0) {
		return M1;
	}

	int first = holeBoundaryEdgesSphere.begin()->first;
	int end = first;
	int current = first;


	std::vector<int> lastMapping;
	std::vector<int> pointsWithoutPartner;

	bool started = false;

	// adding the vertices of the sphere
	for (const auto& triangle : ico_remaining) {
		//R.start_face();

		int index1;
		int index2;
		int index3;

		if (ico_vertex_index_mapping.find(triangle.v1) == ico_vertex_index_mapping.end()) {
			// not yet mapped
			ico_vertex_index_mapping.insert(std::make_pair(triangle.v1, R.get_nr_positions()));
			//R.new_corner(R.get_nr_positions());
			index1 = R.get_nr_positions();
			R.new_position(IS.GetVertices()[triangle.v1]);
		}
		else {
			auto index = ico_vertex_index_mapping.at(triangle.v1);
			//R.new_corner(index);
			index1 = index;
		}

		if (ico_vertex_index_mapping.find(triangle.v2) == ico_vertex_index_mapping.end()) {
			// not yet mapped
			ico_vertex_index_mapping.insert(std::make_pair(triangle.v2, R.get_nr_positions()));
			//R.new_corner(R.get_nr_positions());
			index2 = R.get_nr_positions();
			R.new_position(IS.GetVertices()[triangle.v2]);
		}
		else {
			auto index = ico_vertex_index_mapping.at(triangle.v2);
			//R.new_corner(index);
			index2 = index;
		}

		if (ico_vertex_index_mapping.find(triangle.v3) == ico_vertex_index_mapping.end()) {
			// not yet mapped
			ico_vertex_index_mapping.insert(std::make_pair(triangle.v3, R.get_nr_positions()));
			//R.new_corner(R.get_nr_positions());
			index3 = R.get_nr_positions();
			R.new_position(IS.GetVertices()[triangle.v3]);
		}
		else {
			auto index = ico_vertex_index_mapping.at(triangle.v3);
			//R.new_corner(index);
			index3 = index;
		}

		create_face(R, index1, index2, index3, op != CSG_Operation::CSG_DIFFERENCE);
	}

	// adding the vertices of the mesh
	for (const auto& face : mesh_remaining) {
		auto vertices = HE.GetVerticesForFace(face);
		R.start_face();


		for (const auto& vertex : vertices) {
			if (mesh_vertex_index_mapping.find(vertex->originalIndex) == mesh_vertex_index_mapping.end()) {
				// not yet mapped
				mesh_vertex_index_mapping.insert(std::make_pair(vertex->originalIndex, R.get_nr_positions()));
				R.new_corner(R.get_nr_positions());
				R.new_position(HE.GetOriginalIndexMapping()->at(vertex->originalIndex)->position);
			}
			else {
				auto index = mesh_vertex_index_mapping.at(vertex->originalIndex);
				R.new_corner(index);
			}
		}
	}

	// closing the meshes

	std::cout << "BOUNDARY EDGES SPHERE: " << holeBoundaryEdgesSphere.size() << std::endl;
	std::cout << "BOUNDARY EDGES MESH:   " << holeBoundaryEdgesMesh.size() << std::endl;

	//break;

	for (unsigned i = 0; i < holeBoundaryEdgesMesh.size(); i++) {
		auto v1 = holeBoundaryEdgesMesh[i]->origin->position;
		auto v2 = holeBoundaryEdgesMesh[i]->next->origin->position;
	
		auto middle = ((v2 - v1) * .5f) + v1;

		auto minDistance = 99999.9f;
		int minVertex;

		for (auto entry : holeBoundaryEdgesSphere) {
			auto vertex = IS.GetVertices()[entry.first];
			auto distance = (middle - vertex).length();
			if (distance < minDistance) {
				minDistance = distance;
				minVertex = entry.first;
			}
		}

		if (holeCloseMapping.find(minVertex) == holeCloseMapping.end()) {
			holeCloseMapping.insert(std::make_pair(minVertex, std::vector<int>()));
		}

		auto indexV1 = holeBoundaryEdgesMesh[i]->origin->originalIndex;
		auto indexV2 = holeBoundaryEdgesMesh[i]->next->origin->originalIndex;

		holeCloseMapping.at(minVertex).push_back(indexV1);
		holeCloseMapping.at(minVertex).push_back(indexV2);



		R.start_face();

		if (mesh_vertex_index_mapping.find(indexV1) == mesh_vertex_index_mapping.end()) {
			// not yet mapped
			mesh_vertex_index_mapping.insert(std::make_pair(indexV1, R.get_nr_positions()));
			R.new_corner(R.get_nr_positions());
			R.new_position(HE.GetOriginalIndexMapping()->at(indexV1)->position);
		}
		else {
			auto index = mesh_vertex_index_mapping.at(indexV1);
			R.new_corner(index);
		}

		if (mesh_vertex_index_mapping.find(indexV2) == mesh_vertex_index_mapping.end()) {
			// not yet mapped
			mesh_vertex_index_mapping.insert(std::make_pair(indexV2, R.get_nr_positions()));
			R.new_corner(R.get_nr_positions());
			R.new_position(HE.GetOriginalIndexMapping()->at(indexV2)->position);
		}
		else {
			auto index = mesh_vertex_index_mapping.at(indexV2);
			R.new_corner(index);
		}

		//R.new_corner(mesh_vertex_index_mapping.at(indexV1));
		//R.new_corner(mesh_vertex_index_mapping.at(indexV2));

		if (ico_vertex_index_mapping.find(minVertex) == ico_vertex_index_mapping.end()) {
			// not yet mapped
			ico_vertex_index_mapping.insert(std::make_pair(minVertex, R.get_nr_positions()));
			R.new_corner(R.get_nr_positions());
			R.new_position(IS.GetVertices()[minVertex]);
		}
		else {
			auto index = ico_vertex_index_mapping.at(minVertex);
			R.new_corner(index);
		}
		//R.new_corner(ico_vertex_index_mapping.at(minVertex));
	}

	do {
		if (started) {
			pointsWithoutPartner.push_back(current);
			if (holeCloseMapping.find(current) != holeCloseMapping.end()) {
				auto currentMapping = holeCloseMapping.at(current);
				int overlap = -1;
				for (int v1 : lastMapping) {
					for (int v2 : currentMapping) {
						if (v1 == v2) {
							overlap = v1;
							break;
						}
					}
				}
				for (int i = 0; i < pointsWithoutPartner.size() - 1; i++) {
					/*R.start_face();
					R.new_corner(ico_vertex_index_mapping.at(pointsWithoutPartner[i]));
					R.new_corner(mesh_vertex_index_mapping.at(overlap));
					R.new_corner(ico_vertex_index_mapping.at(pointsWithoutPartner[i+1]));*/

					int index1 = ico_vertex_index_mapping.at(pointsWithoutPartner[i]);
					int index2 = ico_vertex_index_mapping.at(pointsWithoutPartner[i + 1]);
					int index3 = mesh_vertex_index_mapping.at(overlap);

					create_face(R, index1, index2, index3, op != CSG_Operation::CSG_DIFFERENCE);
				}
				pointsWithoutPartner.clear();
				pointsWithoutPartner.push_back(current);
				lastMapping = currentMapping;
			}
		}
		else {
			if (holeCloseMapping.find(current) != holeCloseMapping.end()) {
				started = true;
				lastMapping = holeCloseMapping.at(current);
				first = current;
				pointsWithoutPartner.push_back(current);
			}
		}

		if (current == holeBoundaryEdgesSphere.at(first)) {
			end = current;
		}

		current = holeBoundaryEdgesSphere.at(current);
	} while (current != end);


	std::cout << " HERE WE ARE NOW n " << std::endl;

	R.compute_vertex_normals();
	return R;
}