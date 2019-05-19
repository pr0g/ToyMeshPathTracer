#include "scene.h"
#include <string>
#include <vector>
#include <memory>

#include "tbb/scalable_allocator.h"
#include "tbb/cache_aligned_allocator.h"

// Checks if one triangle is hit by a ray segment.
static bool HitTriangle(const Ray& r, const Triangle& tri, float tMin, float tMax, Hit& outHit)
{
    glm::vec3 edge0 = tri.v1 - tri.v0;
    glm::vec3 edge1 = tri.v2 - tri.v1;
    glm::vec3 normal = normalize(cross(edge0, edge1));
    float planeOffset = dot(tri.v0, normal);

    glm::vec3 p0 = r.pointAt(tMin);
    glm::vec3 p1 = r.pointAt(tMax);

    float offset0 = dot(p0, normal);
    float offset1 = dot(p1, normal);

    // does the ray segment between tMin & tMax intersect the triangle plane?
    if ((offset0 - planeOffset) * (offset1 - planeOffset) <= 0.0f)
    {
        float t = tMin + (tMax - tMin)*(planeOffset - offset0) / (offset1 - offset0);
        glm::vec3 p = r.pointAt(t);

        glm::vec3 c0 = cross(edge0, p - tri.v0);
        glm::vec3 c1 = cross(edge1, p - tri.v1);
        if (dot(c0, c1) >= 0.f)
        {
            auto edge2 = tri.v0 - tri.v2;
            auto c2 = cross(edge2, p - tri.v2);
            if (dot(c1, c2) >= 0.0f)
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
