#pragma once
#include "simple_csg.h"


mesh_type SimpleCSG::compute_intersections(mesh_type& M1, HE_Mesh& HE, AabbTree<triangle>& aabb_tree, IcoSphere& IS, CSG_Operation op) {
	vec3 icosphere_center = IS.GetSphereCenter();
	float radius = IS.GetSphereRadius();

	std::map<vec3*, bool> meshPointInIcosphere;
	std::map<HE_Edge*, CSG_Intersect_Region> edge_regions;
	std::map<HE_Edge*, vec3> intersect_points;

	std::map<vec3*, bool> icoPointInMesh;

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

	for (vec3 p : IS.GetVertices())
	{
		ray_intersection::ray ray = ray_intersection::ray(icosphere_center, p - icosphere_center);
		//ray_intersection::ray tes_ray = ray_intersection::ray(origin, direction);
		float t = 0.0;
		bool intersect = ray_intersection::rayTreeIntersect(ray, aabb_tree, t);

		if (!intersect) {
			icoPointInMesh.insert(std::make_pair(&p, false));
		}
		else {
			icoPointInMesh.insert(std::make_pair(&p, t < 1.0f));
		}
	}

	std::vector<IcoSphere::TriangleIndices> ico_remaining;

	bool ico_in_mesh_remain = (op == CSG_Operation::CSG_DIFFERENCE || op == CSG_Operation::CSG_INTERSECTION);
	bool ico_out_mesh_remain = (op == CSG_Operation::CSG_UNION);
	bool mesh_in_ico_remain = (op == CSG_Operation::CSG_INTERSECTION);
	bool mesh_out_ico_remain = (op == CSG_Operation::CSG_DIFFERENCE || op == CSG_Operation::CSG_UNION);

	for (IcoSphere::TriangleIndices t : IS.GetTriangles()) { 
		bool v1_inside = icoPointInMesh.at(&(IS.GetVertices().at(t.v1)));
		bool v2_inside = icoPointInMesh.at(&(IS.GetVertices().at(t.v2)));
		bool v3_inside = icoPointInMesh.at(&(IS.GetVertices().at(t.v3)));

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
		else {
			//TODO do this
		}
	}

	std::vector<HE_Face*> mesh_remaining;

	for (HE_Face* f : *HE.GetFaces()) {
		HE_Edge* originalEdge = f->adjacent;
		HE_Edge* edge = originalEdge;

		bool all_outside = true;
		bool all_inside = true;

		do {

			edge = edge->next;
		} while (edge != originalEdge);
	}

	//TODO return new built mesh!!!
	mesh_type R;
	return R;
}

mesh_type SimpleCSG::Subtract(mesh_type& M1, IcoSphere& M2) {
	// the result
	mesh_type R;

	// TODO actual code

	return R;
}