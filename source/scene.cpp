#include "scene.h"
#include <string>
#include <vector>
#include <memory>

#include "tbb/scalable_allocator.h"
#include "tbb/cache_aligned_allocator.h"

#include <xmmintrin.h>

#include "external/rtm/vector4f.h"

// Checks if one triangle is hit by a ray segment.
static bool HitTriangle(const Ray& r, const Triangle& tri, float tMin, float tMax, Hit& outHit)
{
    rtm::vector4f edge0 = rtm::vector_sub(tri.v1, tri.v0);
    rtm::vector4f edge1 = rtm::vector_sub(tri.v2, tri.v1);
    rtm::vector4f normal =
        rtm::vector_normalize3(
            rtm::vector_cross3(edge0, edge1),
            rtm::vector_set(0.0f));
    float planeOffset = rtm::vector_dot3(tri.v0, normal);

    rtm::vector4f p0 = r.pointAt(tMin);
    rtm::vector4f p1 = r.pointAt(tMax);

    float offset0 = rtm::vector_dot3(p0, normal);
    float offset1 = rtm::vector_dot3(p1, normal);

    // does the ray segment between tMin & tMax intersect the triangle plane?
    if ((offset0 - planeOffset) * (offset1 - planeOffset) <= 0.0f)
    {
        float t = tMin + (tMax - tMin)*(planeOffset - offset0) / (offset1 - offset0);
        rtm::vector4f p = r.pointAt(t);

        rtm::vector4f c0 =
            rtm::vector_cross3(
                edge0,
                rtm::vector_sub(p, tri.v0));
        
        rtm::vector4f c1 =
            rtm::vector_cross3(
                edge1,
                rtm::vector_sub(p, tri.v1));
        
        if (rtm::vector_dot3(c0, c1) >= 0.f)
        {
            rtm::vector4f edge2 = rtm::vector_sub(tri.v0, tri.v2);
            rtm::vector4f c2 =
                rtm::vector_cross3(
                    edge2,
                    rtm::vector_sub(p, tri.v2));
            
            if (rtm::vector_dot(c1, c2) >= 0.0f)
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

struct Scene
{
    Scene(int triangleCount, const Triangle* triangles)
    {
        m_Triangles.assign(triangles, triangles + triangleCount);
    }
    
    // need bvh structure to manage...

    // Scene information: just a copy of the input triangles.
    std::vector<Triangle, tbb::cache_aligned_allocator<Triangle>> m_Triangles;
};

std::unique_ptr<Scene> scene;

void InitializeScene(int triangleCount, const Triangle* triangles)
{
    scene = std::make_unique<Scene>(triangleCount, triangles);
}

// Check all the triangles in the scene for a hit, and return the closest one.
int HitScene(const Ray& r, float tMin, float tMax, Hit& outHit)
{
    float hitMinT = tMax;
    int hitID = -1;
    for (int i = 0; i < scene->m_Triangles.size(); ++i)
    {
        Hit hit;
        if (HitTriangle(r, scene->m_Triangles[i], tMin, tMax, hit))
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
