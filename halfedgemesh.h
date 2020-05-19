#pragma once

#include <vector>
#include <map>
#include <cgv\math\fvec.h>

/// type of 3d vector
typedef typename cgv::math::fvec<float, 3> vec3;

struct HE_Vertex;
struct HE_Face;

struct HE_Edge {
	/// the origin vertex of the halfedge
	HE_Vertex* origin;
	/// the face the halfedge belongs to
	HE_Face* face;
	/// the next halfedge around the face
	HE_Edge* next;
	/// twin halfedge
	HE_Edge* twin = nullptr;
};

struct HE_Vertex {
	/// position of the vertex
	vec3 position;
	/// outgoing halfegde for the vertex
	HE_Edge* outgoing;
	/// original index for the vertex (from simple_mesh)
	unsigned int originalIndex;
};

struct HE_Face {
	/// one adjacent halfegde of the mesh, needed to find adjacent faces
	HE_Edge* adjacent = nullptr;
	/// one half-edge of the face;
	HE_Edge* face_edge;

};

class HE_Mesh
{
public:
	HE_Mesh() {}
	~HE_Mesh() {
		vertices.clear();
		halfEdges.clear();
		faces.clear();
		boundaryFaces.clear();

		originalVectorIndices.clear();
		originalEdges.clear();
	}

	/// return reference to the stored vertices of the mesh
	std::vector<HE_Vertex*>* GetVertices() { return &vertices; }
	/// return reference to the stored halfedges of the mesh
	std::vector<HE_Edge*>* GetHalfEdges() { return &halfEdges; }
	/// return reference to the stored faces of the mesh
	std::vector<HE_Face*>* GetFaces() { return &faces; }
	/// return reference to the stored boundaryfaces of the mesh
	std::vector<HE_Face*>* GetBoundaryFaces() { return &boundaryFaces; }

	std::map<unsigned int, HE_Vertex*> GetOriginalVectorIndices() { return originalVectorIndices; }

	/// adding a vector and returning a reference to it, checks for duplicates based on originalIndex
	HE_Vertex* AddVector(unsigned int originalIndex, vec3 position);
	/// adding a face and returning a reference to it
	HE_Face* AddFace();
	/// adding a halfedge and returning a reference to it, checks for duplicates
	/// will also set the adjacent halfedge of the face if it is not already set
	HE_Edge* AddHalfEdge(HE_Vertex* origin, HE_Vertex* dest, HE_Face* face, HE_Edge* next = nullptr);

	/// return adjacent faces for a given one in the mesh (for now only triangles supported)
	std::vector<HE_Face*> GetAdjacentFaces(HE_Face* face);

	/// return true if the mesh is closed
	bool isClosed() { return boundaryFaces.empty() ? true : false; }

	HE_Face* AddBoundary(HE_Edge* edge);

private:
	std::vector<HE_Vertex*> vertices;
	std::vector<HE_Edge*> halfEdges;
	std::vector<HE_Face*> faces;
	std::vector<HE_Face*> boundaryFaces;

	std::map<unsigned int, HE_Vertex*> originalVectorIndices;
	std::map<std::pair<unsigned int, unsigned int>, HE_Edge*> originalEdges;
};