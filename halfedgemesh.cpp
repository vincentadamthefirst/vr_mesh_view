#include "halfedgemesh.h"

HE_Face* HE_Mesh::AddFace() {
	auto newFace = new HE_Face;
	faces.push_back(newFace);
	return newFace;
}

HE_Vertex* HE_Mesh::AddVector(unsigned int originalIndex, vec3 position) {
	auto foundResult = originalVectorIndices.find(originalIndex);
	if (foundResult != originalVectorIndices.end()) {
		return foundResult->second;
	}

	auto newVertex = new HE_Vertex;
	newVertex->originalIndex = originalIndex;
	newVertex->position = position;
	vertices.push_back(newVertex);
	originalVectorIndices.insert(std::make_pair(originalIndex, newVertex));
	return newVertex;
}

int CantorPairing(int k1, int k2) {
	return ((k1 + k2) * (k1 + k2 + 1) / 2) + k2;
}

HE_Edge* HE_Mesh::AddHalfEdge(HE_Vertex* origin, HE_Vertex* dest, HE_Face* face, HE_Edge* next) {
	auto newHalfEdge = new HE_Edge;
	newHalfEdge->origin = origin;
	newHalfEdge->next = next;
	newHalfEdge->face = face;

	if (face->adjacent == nullptr) face->adjacent = newHalfEdge;

	auto possibleExisting = originalEdges.find(CantorPairing(origin->originalIndex, dest->originalIndex));
	if (possibleExisting != originalEdges.end()) {
		delete newHalfEdge;
		return possibleExisting->second;
	}

	auto possibleTwin = originalEdges.find(CantorPairing(dest->originalIndex, origin->originalIndex));
	if (possibleTwin != originalEdges.end()) {
		newHalfEdge->twin = possibleTwin->second;
		possibleTwin->second->twin = newHalfEdge;
	}
	origin->outgoing = newHalfEdge;
	halfEdges.push_back(newHalfEdge);
	originalEdges.insert(std::make_pair(CantorPairing(origin->originalIndex, dest->originalIndex), newHalfEdge));
	return newHalfEdge;
}

std::vector<HE_Face*> HE_Mesh::GetAdjacentFaces(HE_Face* face) {
	std::vector<HE_Face*> toReturn;

	if (face->adjacent->twin != nullptr && face->adjacent->twin->face != nullptr) {
		toReturn.push_back(face->adjacent->twin->face);
	}
	if (face->adjacent->next->twin != nullptr && face->adjacent->next->twin->face != nullptr) {
		toReturn.push_back(face->adjacent->next->twin->face);
	}
	if (face->adjacent->next->next->twin != nullptr && face->adjacent->next->next->twin->face != nullptr) {
		toReturn.push_back(face->adjacent->next->next->twin->face);
	}

	return toReturn;
}

HE_Face* HE_Mesh::AddBoundary(HE_Edge* edge) {
	HE_Vertex* v = edge->origin;
	boundaryVertices.push_back(v);
	HE_Face* f = edge->face;
	// f is not a element of boundaryFaces
	if (std::find(boundaryFaces.begin(), boundaryFaces.end(), f) == boundaryFaces.end())
		boundaryFaces.push_back(f);
	return f;
}

std::vector<HE_Vertex*> HE_Mesh::GetVerticesForFace(HE_Face* face) {
	std::vector<HE_Vertex*> toReturn;

	toReturn.push_back(face->adjacent->origin);
	toReturn.push_back(face->adjacent->next->origin);
	toReturn.push_back(face->adjacent->next->next->origin);

	return toReturn;
}



std::vector<HE_Vertex*> HE_Mesh::GetNeighborVertices(HE_Vertex* vertex) {
	bool loopReverse = false;
	std::vector<HE_Vertex*> toReturn;
	HE_Edge* e1 = vertex->outgoing;
	auto e2 = e1->twin;
	//boundary
	if (e2 == nullptr) {
		// loop in andere Richtung#
		loopReverse = true;
	}
	
	int i = 0;
	//normal loop
	if (!loopReverse) {
		HE_Edge* start = e2;
		do {
			toReturn.push_back(e2->origin);
			auto e2_temp = e2->next->twin;
			//boundary
			if (e2_temp == nullptr) {
				// loop in andere Richtung
				toReturn.push_back(e2->next->next->origin);
				loopReverse = true;
				break;
			}
			else {
				e2 = e2_temp;
			}
			//std::cout << i++ << std::endl;
		} while (e2 != start);
	}
	
	//loop reverse, used if there is a boundary 
	if (loopReverse) {
		e2 = e1->next->next;
		while(true) {
			toReturn.push_back(e2->origin);
			if (e2->twin == nullptr)
				break;
			e2 = e2->twin->next->next;
			//std::cout << i++ << std::endl;
		} 

	}
	return toReturn;
}

//Why vertices of he_mesh and simple mesh do not have the same originalIndex values pointing for the same vertex position? 
//They are not compatible with each other
//Therefore this more efficient version does not work
/*
bool HE_Mesh::changeVertexPos(HE_Vertex* vertex, vec3 new_pos) {
	if (vertex == vertices[vertex->originalIndex]) {
		//std::cout << "Vertex "<< vertices[vertex->originalIndex]->originalIndex <<" with position "<< vertices[vertex->originalIndex]->position << " changed to "<< vertex << " with position "<< vertex->position <<std::endl;
		vertices[vertex->originalIndex]->position = new_pos;
		return true;
	}
	return false;
}
*/

bool HE_Mesh::changeVertexPos(HE_Vertex* vertex, vec3 new_pos) {

	for (auto v : vertices) {
		if (v->position == vertex->position) {
			//std::cout << "Vertex " << v->originalIndex << " with position " << v->position << " changed to " << "vertex intersection point" << " with position " << new_pos << std::endl;
			v->position = new_pos;
			return true;
		}
	}
	return false;
}
