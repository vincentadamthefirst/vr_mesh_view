#include "halfedgemesh.h"
using namespace cgv::math;
typedef typename fvec<float, 3> vec3;

namespace mesh_utils {  
    void getVerticesOfFace(HE_Face* face, vec3& p1, vec3& p2, vec3& p3) {
        p1 = face->adjacent->origin->position;
        p2 = face->adjacent->next->origin->position;
        p3 = face->adjacent->next->next->origin->position;
    }
    float triangle_area(vec3 p1, vec3 p2, vec3 p3) {
        vec3 a = p1 - p2;
        vec3 b = p3 - p2;
        return cross(a, b).length() / 2.0f;
    }

    float surface(HE_Mesh *newMesh) {
        float result = 0.0f;
        vec3 p1, p2, p3;
        for (auto face : *newMesh->GetFaces()) {
            getVerticesOfFace(face, p1, p2, p3);
            float area = triangle_area(p1,p2,p3);
            result += area;
        }
        return result;
    }

    float signed_volume_tetrahedron(vec3 p1, vec3 p2, vec3 p3) {
        return dot(p1, cross(p2, p3)) / 6.0f;
    }

    float volume(HE_Mesh *newMesh) {
        if (!newMesh->isClosed()) return -1;
        float result = 0;
        vec3 p1, p2, p3;
        for (auto face : *newMesh->GetFaces()) {
            getVerticesOfFace(face, p1, p2, p3);
            result += signed_volume_tetrahedron(p1, p2, p3);
        }
        return result;
    }    
   
    //relative position on line ... proj == a --> retrun 0, proj ==b --> return 1
    //proj needs to be a point on the line 
    float position_on_line(vec3 a, vec3 b, vec3 proj) {
        vec3 t = proj - a;
        vec3 s = b - a;
        // s and t are pointing in the same direction
        if (dot(t,s) > 0)
            return t.length()/s.length();
        else
            return -t.length() / s.length();
    }
    // p is projected onto line ab
    vec3 projection_onto_line(vec3 a, vec3 b, vec3 p) {
        vec3 ap = p - a;
        vec3 ab = b - a;
        return a + dot(ap, ab) / dot(ab, ab) * ab;
    }

    // gives back the closest point on edge --> vertex or projected edgepoint
    vec3 closest_point_on_edge(vec3 a, vec3 b, vec3 p) {
        vec3 proj = projection_onto_line(a, b, p);
        float pos = position_on_line(a, b, proj);

        if (pos >= 0 && pos <= 1)
            return proj;
        else if (pos < 0)
            return a;
        else
            return b;
    }

    // the closest point from point p on triangle (a,b,c) is returned
    vec3 closest_point_on_triangle(vec3 p, vec3 a, vec3 b, vec3 c) {
        // Find normal to plane n = (b - a) x (c - a)
        vec3 n = cross(b - a, c - a);
        n.normalize();
        // project point onto plane given by n and a
        // distance from p to plane
        float dist = dot(p, n) - dot(a, n);
        // Project p onto the plane by stepping the distance from p to the plane
        // in the direction opposite the normal: proj = p - dist * n
        vec3 proj = p - dist * n;

        // check if point falls into triangle 
         // Compute edge vectors        
        vec3 v0 = c - a;
        vec3 v1 = b - a;
        vec3 v2 = proj - a;

        // Compute dot products
        float dot00 = dot(v0, v0);
        float dot01 = dot(v0, v1);
        float dot02 = dot(v0, v2);
        float dot11 = dot(v1, v1);
        float dot12 = dot(v1, v2);

        // Compute barycentric coordinates (u, v) of projection point
        float denom = (dot00 * dot11 - dot01 * dot01);
        float invDenom = 1.0f / denom;
        float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
        float w = 1.0f - v - u;

        // Check barycentric coordinates
        if ((u >= 0) && (v >= 0) && (u + v <= 1)) {
            // Nearest orthogonal projection point is in triangle
            return proj;
        }
        else {
            // Nearest orthogonal projection point is outside triangle
            // project proj onto edges and vertices
            if (u < 0 && v >= 0 && w >= 0) {
                // edge ab
                return closest_point_on_edge(a, b, p);
            }
            if (u >= 0 && v < 0 && w >= 0) {
                // edge ca
                return closest_point_on_edge(c, a, p);
            }
            if (u >= 0 && v >= 0 && w < 0) {
                // adge bc
                return closest_point_on_edge(b, c, p);
            }
            if (u >= 0 && v < 0 && w < 0) {
                //area around vertex c
                vec3 point1 = closest_point_on_edge(c, b, p);
                vec3 point2 = closest_point_on_edge(c, a, p);
                if ((point1 - p).length() < (point2 - p).length())
                    return point1;
                else
                    return point2;
            }
            if (u < 0 && v >= 0 && w < 0) {
                //area around vertex b
                vec3 point1 = closest_point_on_edge(a, b, p);
                vec3 point2 = closest_point_on_edge(c, b, p);
                if ((point1 - p).length() < (point2 - p).length())
                    return point1;
                else
                    return point2;
            }
            if (u < 0 && v < 0 && w >= 0) {
                //area around vertex a
                vec3 point1 = closest_point_on_edge(a, b, p);
                vec3 point2 = closest_point_on_edge(a, c, p);
                if ((point1 - p).length() < (point2 - p).length())
                    return point1;
                else
                    return point2;
            }
        }

    }

    float shortest_distance(vec3 point, HE_Mesh* newMesh, HE_Face* & closestFace, vec3& closestPoint) {
        float min_dist = std::numeric_limits<float>::max();
        vec3 p1, p2, p3;
        closestFace;
        //brute force with middle of triangle
        for (auto face : *newMesh->GetFaces()) {
            getVerticesOfFace(face, p1, p2, p3);
            //vec3 middle = (p1 + p2 + p3) / 3.0f;
            closestPoint = closest_point_on_triangle(point, p1, p2, p3);
            float distance = (closestPoint - point).length();
            if (distance < min_dist) {
                min_dist = distance;
                closestFace = face;
            }

        }
        //TODO
        //with acceleration structure

        return min_dist;
    }    
}