#pragma once
#include <cgv\math\fvec.h>
#include "halfedgemesh.h"
#include "mesh_utils.h"
typedef typename cgv::math::fvec<float, 3> vec3;


namespace ray_intersection {
	struct ray {
		vec3 origin;
		vec3 direction;
	};
	bool isInsideTriangle(const vec3& v0, const vec3& v1, const vec3& v2, const vec3& p)
	{
		vec3 edge0 = v1 - v0;
		vec3 edge1 = v2 - v1;
		vec3 edge2 = v0 - v2;
		vec3 C0 = p - v0;
		vec3 C1 = p - v1;
		vec3 C2 = p - v2;
		vec3 N = cross(edge0, edge1);
		if (dot(N,cross(edge0,C0)) > 0 &&
			dot(N, cross(edge1, C1)) > 0 &&
			dot(N, cross(edge2, C2)) > 0) return true; // P is inside the triangle 
		return false;
	}

	/*
	Doesnt work
	bool old_rayTriangleIntersect(const ray& r, const vec3& v0, const vec3& v1, const vec3& v2, float& t)
	{
		vec3 dir = r.direction;
		vec3 orig = r.origin;

		// compute plane's normal
		vec3 v0v1 = v1 - v0;
		vec3 v0v2 = v2 - v0;
		// no need to normalize
		vec3 N = cross(v0v1, v0v2);
		// Step 1: finding P

		// check if ray and plane are parallel ?
		float NdotRayDirection = dot(N, dir);
		if (fabs(NdotRayDirection) < std::numeric_limits<float>::epsilon()) // almost 0 
			return false; // they are parallel so they don't intersect ! 

		// compute d parameter
		float d = dot(N, v0);

		// compute t
		t = (dot(N, orig) + d) / NdotRayDirection;
		// check if the triangle is in behind the ray
		if (t < 0) return false; // the triangle is behind 

		// compute the intersection point using equation 1
		vec3 P = orig + t * dir;

		// Step 2: inside-outside test
		vec3 C; // vector perpendicular to triangle's plane 

		// edge 0
		vec3 edge0 = v1 - v0;
		vec3 vp0 = P - v0;
		C = cross(edge0, vp0);
		if (dot(N, C) < 0) return false; // P is on the right side 

		// edge 1
		vec3 edge1 = v2 - v1;
		vec3 vp1 = P - v1;
		C = cross(edge1, vp1);
		if (dot(N, C) < 0)  return false; // P is on the right side 

		// edge 2
		vec3 edge2 = v0 - v2;
		vec3 vp2 = P - v2;
		C = cross(edge2, vp2);
		if (dot(N, C) < 0) return false; // P is on the right side; 

		return true; // this ray hits the triangle 
	}
	*/

	//t value gives the distance multiplier 
	bool rayTriangleIntersect(const ray& r, const vec3& v0, const vec3& v1, const vec3& v2, float& t)
	{
		vec3 d = r.direction;
		vec3 o = r.origin;
		vec3 e1 = v1 - v0;
		vec3 e2 = v2 - v0;
		vec3 p = cross(d, e2);
		float a = dot(e1, p);

		if (a == 0)
			return false;

		float f = 1.0f / a;

		vec3 s = o - v0;
		float u = f * dot(s, p);
		if (u < 0.0f || u > 1.0f)
			return false;

		vec3 q = cross(s, e1);
		float v = f * dot(d, q);

		if (v < 0.0f || u + v > 1.0f)
			return false;

		t = f * dot(e2, q);

		return (t >= 0);
	}
	
	bool rayFaceIntersect(ray& r, HE_Mesh* mesh, HE_Face* face, float& t)
	{
		vec3 v0, v1, v2;
		mesh_utils::getVerticesOfFace(mesh, face, v0, v1, v2);
		if (rayTriangleIntersect(r, v0, v1, v2, t)) {
			return true;
		}
			
		return false;
	}
	
	bool rayMeshIntersect(ray& r, HE_Mesh* mesh, float& t)
	{
		vec3 v0, v1, v2;
		bool intersect = false;
		float temp_t = 1000000;

		for (auto face : *mesh->GetFaces()) {
			if (rayFaceIntersect(r, mesh, face, t))
			{
				intersect = true;
				if (t < temp_t)
					temp_t = t;
			}
		}
		
		if (!intersect)
			return false;
		else {
			t = temp_t;
			return true;
		}
	}
	HE_Face* getIntersectedFace(ray& r, HE_Mesh* mesh)
	{
		vec3 v0, v1, v2;
		bool intersect = false;
		float temp_t = 1000000;
		float t = 0;

		HE_Face* temp_face = new HE_Face();

		for (auto face : *mesh->GetFaces()) {
			if (rayFaceIntersect(r, mesh, face, t))
			{
				intersect = true;
				if (t < temp_t) {
					temp_t = t;
					temp_face = face;
				}		
			}
		}
		if (intersect)
			return temp_face;
		else {
			std::cout << "No intersection, returning face object anyway" << std::endl;
				return temp_face;
		}
	}
	vec3 getIntersectionPoint(const ray& r, const float& t) 
	{
		return r.origin + t * r.direction;
	}
	
}