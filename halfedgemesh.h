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
	vec3 normal = vec3(1, 0, 0);
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
	/// returns the mapping for the original indices of the stored vertices
	std::map<unsigned int, HE_Vertex*>* GetOriginalIndexMapping() { return &originalVectorIndices; }

	/// adding a vector and returning a reference to it, checks for duplicates based on originalIndex
	HE_Vertex* AddVector(unsigned int originalIndex, vec3 position);
	/// adding a face and returning a reference to it
	HE_Face* AddFace();
	/// adding a halfedge and returning a reference to it, checks for duplicates
	/// will also set the adjacent halfedge of the face if it is not already set
	HE_Edge* AddHalfEdge(HE_Vertex* origin, HE_Vertex* dest, HE_Face* face, HE_Edge* next = nullptr);

	/// return adjacent faces for a given one in the mesh (for now only triangles supported)
	std::vector<HE_Face*> GetAdjacentFaces(HE_Face* face);
	/// return adjacent faces for a given vertex
	std::vector<HE_Face*> GetAdjacentFaces(HE_Vertex* vertex);
	/// return vertices making up a face (for now only triangles supported)
	std::vector<HE_Vertex*> GetVerticesForFace(HE_Face* face);
	/// returns neighbor vertices for a given Vertex
	std::vector<HE_Vertex*> GetNeighborVertices(HE_Vertex* vertex);

	// changes the position of a vertex
	bool changeVertexPos(HE_Vertex* vertex, vec3 new_pos);

	/// return true if the mesh is closed
	bool isClosed() { return boundaryFaces.empty() ? true : false; }
	/// adds a boundary Face for the given He_Edge to the vector of boundaryFaces, returns the Face 
	HE_Face* AddBoundary(HE_Edge* edge);
	// deletes the given Face and corresponding edges from the He_mesh
	bool deleteFace(HE_Face* f);
	//deletes the given vertex
	bool deleteVector(HE_Vertex* vertex);

private:
	std::vector<HE_Vertex*> vertices;
	std::vector<HE_Edge*> halfEdges;
	std::vector<HE_Face*> faces;
	std::vector<HE_Face*> boundaryFaces;
	std::vector<HE_Vertex*> boundaryVertices;

	std::map<unsigned int, HE_Vertex*> originalVectorIndices;
	std::map<int, HE_Edge*> originalEdges;
};
