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
        float result = 0;
        for (auto face : *newMesh.GetFaces()) {
            vec3 p1 = face->adjacent->origin->position;
            vec3 p2 = face->adjacent->next->origin->position;
            vec3 p3 = face->adjacent->next->next->origin->position;
            result += triangle_area(p1,p2,p3);
        }
        return result;
    }

    float signed_volume_tetrahedron(vec3 p1, vec3 p2, vec3 p3) {
        return dot(p1, cross(p2, p3)) / 6.0f;
    }


    float volume(HE_Mesh newMesh) {
        bool notclosed = false;
        if (notclosed) return -1;
        float result = 0;
        for (auto face : *newMesh.GetFaces()) {
            vec3 p1 = face->adjacent->origin->position;
            vec3 p2 = face->adjacent->next->origin->position;
            vec3 p3 = face->adjacent->next->next->origin->position;
            result += signed_volume_tetrahedron(p1, p2, p3);
        }
        return result;
    }

    float shortest_distance(vec3 point, HE_Mesh newMesh) {
        float min_dist = std::numeric_limits<float>::max();

        //brute force
        for (auto it : *newMesh.GetVertices) {
            float distance = (it->position - point).length();
            if (distance < min_dist)
                min_dist = distance;
        }

        //with acceleration structure


        return min_dist;
    }

}