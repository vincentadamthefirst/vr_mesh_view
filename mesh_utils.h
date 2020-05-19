#include "halfedgemesh.h"
using namespace cgv::math;
typedef typename fvec<float, 3> vec3;

namespace mesh_utils {  

    float triangle_area(vec3 p1, vec3 p2, vec3 p3) {
        vec3 a = p1 - p2;
        vec3 b = p3 - p2;
        return cross(a, b).length() / 2.0f;
    }

    float surface(HE_Mesh newMesh) {
        float result = 0.0f;
        vec3 p1, p2, p3;
        float faceSize = newMesh.GetFaces()->size();
        for (auto face : *newMesh.GetFaces()) {
            getVerticesOfFace(face, p1, p2, p3);
            float area = triangle_area(p1,p2,p3);
            result += area;
        }
        return result;
    }

    float signed_volume_tetrahedron(vec3 p1, vec3 p2, vec3 p3) {
        return dot(p1, cross(p2, p3)) / 6.0f;
    }


    float volume(HE_Mesh newMesh) {
        if (!newMesh.isClosed()) return -1;
        float result = 0;
        vec3 p1, p2, p3;
        for (auto face : *newMesh.GetFaces()) {
            getVerticesOfFace(face, p1, p2, p3);
            result += signed_volume_tetrahedron(p1, p2, p3);
        }
        return result;
    }

    float shortest_distance(vec3 point, HE_Mesh newMesh) {
        float min_dist = std::numeric_limits<float>::max();
        vec3 p1, p2, p3;
        HE_Face *closestFace;
        //brute force with middle of triangle
        for (auto face : *newMesh.GetFaces()) {
            getVerticesOfFace(face, p1, p2, p3);
            vec3 middle = (p1 + p2 + p3) / 3.0f;
            float distance = (middle - point).length();
            if (distance < min_dist) {
                min_dist = distance;
                closestFace = face;
            }
                
        }

        //with acceleration structure


        return min_dist;
    }

    void getVerticesOfFace(HE_Face *face, vec3& p1, vec3& p2, vec3& p3) {
        vec3 p1 = face->face_edge->origin->position;
        vec3 p2 = face->face_edge->next->origin->position;
        vec3 p3 = face->face_edge->next->next->origin->position;
    }
    
}