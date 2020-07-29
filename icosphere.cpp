#include "icosphere.h"

IcoSphere::IcoSphere(float radius, int subdivisions, vec3 center) {

	// setting the center point for later simple_mesh retrieval
	centerPoint = center;

	// setting the icosphere radius for later retrieval
	sphere_radius = radius;

	auto tmpVertList = std::vector<vec3>();
	auto middlePointIndexCache = std::map<int, int>();
	auto index = 0;

	// creating 12 vertices for the icosahedron
	float t = (1.0f + std::sqrt(5.0f)) / 2.0f;

	tmpVertList.push_back(vec3(-1.0f, t, 0.0f));
	tmpVertList.push_back(vec3(1.0f, t, 0.0f));
	tmpVertList.push_back(vec3(-1.0f, -t, 0.0f));
	tmpVertList.push_back(vec3(1.0f, -t, 0.0f));

	tmpVertList.push_back(vec3(0.0f, -1.0f, t));
	tmpVertList.push_back(vec3(0.0f, 1.0f, t));
	tmpVertList.push_back(vec3(0.0f, -1.0f, -t));
	tmpVertList.push_back(vec3(0.0f, 1.0f, -t));

	tmpVertList.push_back(vec3(t, 0.0f, -1.0f));
	tmpVertList.push_back(vec3(t, 0.0f, 1.0f));
	tmpVertList.push_back(vec3(-t, 0.0f, -1.0f));
	tmpVertList.push_back(vec3(-t, 0.0f, 1.0f));

	// scaling these vertices to fit the given radius
	for (auto vertex : tmpVertList) {
		vertex.normalize();
		vertex *= radius;
		vertices.push_back(vertex);
	}

	// create 20 start triangles
	auto faces = std::vector<TriangleIndices>();

	faces.push_back(TriangleIndices(0, 11, 5));
	faces.push_back(TriangleIndices(0, 5, 1));
	faces.push_back(TriangleIndices(0, 1, 7));
	faces.push_back(TriangleIndices(0, 7, 10));
	faces.push_back(TriangleIndices(0, 10, 11));

	faces.push_back(TriangleIndices(1, 5, 9));
	faces.push_back(TriangleIndices(5, 11, 4));
	faces.push_back(TriangleIndices(11, 10, 2));
	faces.push_back(TriangleIndices(10, 7, 6));
	faces.push_back(TriangleIndices(7, 1, 8));

	faces.push_back(TriangleIndices(3, 9, 4));
	faces.push_back(TriangleIndices(3, 4, 2));
	faces.push_back(TriangleIndices(3, 2, 6));
	faces.push_back(TriangleIndices(3, 6, 8));
	faces.push_back(TriangleIndices(3, 8, 9));

	faces.push_back(TriangleIndices(4, 9, 5));
	faces.push_back(TriangleIndices(2, 4, 11));
	faces.push_back(TriangleIndices(6, 2, 10));
	faces.push_back(TriangleIndices(8, 6, 7));
	faces.push_back(TriangleIndices(9, 8, 1));

	// subdivide triangles
	for (int i = 0; i < subdivisions; i++) {
		auto faces2 = std::vector<TriangleIndices>();
		for (auto tri : faces) {
			// replace one triangle by 4 triangles
			int a = GetMiddlePoint(tri.v1, tri.v2, vertices, middlePointIndexCache, radius);
			int b = GetMiddlePoint(tri.v2, tri.v3, vertices, middlePointIndexCache, radius);
			int c = GetMiddlePoint(tri.v3, tri.v1, vertices, middlePointIndexCache, radius);

			faces2.push_back(TriangleIndices(tri.v1, a, c));
			faces2.push_back(TriangleIndices(tri.v2, b, a));
			faces2.push_back(TriangleIndices(tri.v3, c, b));
			faces2.push_back(TriangleIndices(a, b, c));
		}
		faces = faces2;
	}

	triangles = faces;

	for (int i = 0; i < vertices.size(); i++) {
		vertices[i] += center;
	}

	// debug output
	std::cout << "\nGenerated IcoSphere with " << subdivisions << " subdivisions and radius " << radius << std::endl;
	std::cout << "Resulting Vertices:\t" << vertices.size() << std::endl;
	std::cout << "Resulting Faces:\t" << faces.size() << "\n" << std::endl;
}

int CantorPairing2(int k1, int k2) {
	return ((k1 + k2) * (k1 + k2 + 1) / 2) + k2;
}

int IcoSphere::GetMiddlePoint(int p1, int p2, std::vector<vec3>& points, std::map<int, int>& cache, float radius) {
	auto smaller = p1 < p2;
	auto smallerIndex = smaller ? p1 : p2;
	auto greaterIndex = smaller ? p2 : p1;
	auto key = CantorPairing2(smallerIndex, greaterIndex);

	// already in cache, return
	if (cache.find(key) != cache.end()) {
		return cache.at(key);
	}

	// not in cache, calculate
	auto point1 = points[p1];
	auto point2 = points[p2];
	auto middle = vec3((point1.x() + point2.x()) / 2.0f, (point1.y() + point2.y()) / 2.0f, (point1.z() + point2.z()) / 2.0f);

	auto i = points.size();

	// adding the new point (scale it to be on the sphere)
	middle.normalize();
	points.push_back(middle * radius);

	// store in cache, return index
	cache.insert(std::make_pair(key, i));
	return i;
}

mesh_type IcoSphere::RetrieveMesh() {
	mesh_type M;

	for each (auto vertex in vertices) {
		M.new_position(vertex);
	}

	for each (auto triangle in triangles) {
		M.start_face();
		M.new_corner(triangle.v1);
		M.new_corner(triangle.v2);
		M.new_corner(triangle.v3);
	}

	M.compute_vertex_normals();

	// debug output
	std::cout << "\nRetrieved simple_mesh from IcoSphere" << std::endl;
	std::cout << "Vertices:\t" << M.get_nr_positions() << std::endl;
	std::cout << "Normals:\t" << M.get_nr_normals() << std::endl;
	std::cout << "Faces:  \t" << M.get_nr_faces() << "\n" << std::endl;

	return M;
}

vec3 IcoSphere::GetSphereCenter() {
	return centerPoint;
}

float IcoSphere::GetSphereRadius() {
	return sphere_radius;
}

std::vector<vec3>& IcoSphere::GetVertices() {
	return vertices;
}

std::vector<IcoSphere::TriangleIndices>& IcoSphere::GetTriangles() {
	return triangles;
}
