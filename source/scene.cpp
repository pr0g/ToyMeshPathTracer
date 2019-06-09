#include "scene.h"
#include <string.h>


// Checks if one triangle is hit by a ray segment.
static bool HitTriangle(const Ray& r, const Triangle& tri, float tMin, float tMax, Hit& outHit)
{
    as::vec3_t edge0 = tri.v1 - tri.v0;
    as::vec3_t edge1 = tri.v2 - tri.v1;
    as::vec3_t normal = as::vec::normalize(as::vec3::cross(edge0, edge1));
    float planeOffset = as::vec::dot(tri.v0, normal);

    as::vec3_t p0 = r.pointAt(tMin);
    as::vec3_t p1 = r.pointAt(tMax);

    float offset0 = as::vec::dot(p0, normal);
    float offset1 = as::vec::dot(p1, normal);

    // does the ray segment between tMin & tMax intersect the triangle plane?
    if ((offset0 - planeOffset) * (offset1 - planeOffset) <= 0.0f)
    {
        float t = tMin + (tMax - tMin)*(planeOffset - offset0) / (offset1 - offset0);
        as::vec3_t p = r.pointAt(t);

        as::vec3_t c0 = as::vec3::cross(edge0, p - tri.v0);
        as::vec3_t c1 = as::vec3::cross(edge1, p - tri.v1);
        if (as::vec::dot(c0, c1) >= 0.f)
        {
            auto edge2 = tri.v0 - tri.v2;
            auto c2 = as::vec3::cross(edge2, p - tri.v2);
            if (as::vec::dot(c1, c2) >= 0.f)
            {
                outHit.t = t;
                outHit.pos = p;
                outHit.normal = normal;
                return true;
            }
        }
    }

    return false;
}

// Scene information: just a copy of the input triangles.
static int s_TriangleCount;
static Triangle* s_Triangles;


void InitializeScene(int triangleCount, const Triangle* triangles)
{
    s_TriangleCount = triangleCount;
    s_Triangles = new Triangle[triangleCount];
    memcpy(s_Triangles, triangles, triangleCount * sizeof(triangles[0]));
}

void CleanupScene()
{
    delete[] s_Triangles;
}


// Check all the triangles in the scene for a hit, and return the closest one.
int HitScene(const Ray& r, float tMin, float tMax, Hit& outHit)
{
    float hitMinT = tMax;
    int hitID = -1;
    for (int i = 0; i < s_TriangleCount; ++i)
    {
        Hit hit;
        if (HitTriangle(r, s_Triangles[i], tMin, tMax, hit))
        {
            if (hit.t < hitMinT)
            {
                hitMinT = hit.t;
                hitID = i;
                outHit = hit;
            }
        }
    }

    return hitID;
}
