#pragma once

#include <vector>
#include <map>
#include <cgv\math\fvec.h>
#include <cgv\media\mesh\simple_mesh.h>

/// type of 3d vector
typedef typename cgv::math::fvec<float, 3> vec3;
/// type of simple mesh
typedef cgv::media::mesh::simple_mesh<float> mesh_type;

/// Class for generating the vertices and faces of an IcoSphere
class IcoSphere {
public: 
	// represents a triangle face internally
	struct TriangleIndices {
		int v1;
		int v2;
		int v3;

		TriangleIndices(int a, int b, int c) {
			v1 = a;
			v2 = b;
			v3 = c;
		}
	};

public:
	/// Creates an Icosphere with the given radius, subdivisions and center
	IcoSphere(float radius = 1.0f, int subdivisions = 2, vec3 center = vec3(0, 0, 0));

	/// Clears up all used space
	~IcoSphere() {
		vertices.clear();
	}
	
	/// retrieves the IcoSphere as an simple_mesh
	mesh_type RetrieveMesh();

	/// Getter for the IcoShpere's center point
	vec3 GetSphereCenter();

	/// Getter for the IcoShpere's radius
	float GetSphereRadius();

	/// Getter for the IcoShpere's list of vertices
	std::vector<vec3>& GetVertices();

	/// Getter for the IcoShpere's list of vertices
	std::vector<TriangleIndices>& GetTriangles();


private:
	/// calculates the index of the point in the middle of p1 and p2
	static int GetMiddlePoint(int p1, int p2, std::vector<vec3>& points, std::map<long, int>& cache, float radius);

	/// List containing all vertices for the mesh
	std::vector<vec3> vertices;
	/// List containing all edges for the mesh (all triangles)
	std::vector<TriangleIndices> triangles;
	/// The center for this IcoSphere
	vec3 centerPoint;
	/// The radius of this IcoSphere
	float sphere_radius;
};