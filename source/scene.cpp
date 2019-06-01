#include "scene.h"
#include <string>

#include "tbb/parallel_reduce.h"
#include "tbb/blocked_range.h"

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

Scene::Scene(const Triangle* triangles, int triangleCount)
{
    m_triangles.assign(triangles, triangles + triangleCount);
}

void Scene::Cull(const glm::vec3& lookFrom)
{
    m_triangles.erase(
        std::remove_if(
            m_triangles.begin(), m_triangles.end(),
            [lookFrom](const Triangle& tri)
    {
        glm::vec3 edge0 = tri.v1 - tri.v0;
        glm::vec3 edge1 = tri.v2 - tri.v1;
        glm::vec3 normal = normalize(cross(edge0, edge1));
        return glm::dot(tri.v0 - lookFrom, normal) >= 0.0f;
    }), m_triangles.end());
}

void Scene::BuildOctree(const glm::vec3& min, const glm::vec3& max)
{
    m_octree.m_aabb.min = min;
    m_octree.m_aabb.max = max;
    m_octree.m_triangles = m_triangles;
    m_octree.Subdivide();
    for (auto& octree : m_octree.m_children)
    {
        octree->Subdivide();

        for (auto& ot : octree->m_children)
        {
            ot->Subdivide();

            for (auto& t : ot->m_children)
            {
                t->Subdivide();
            }
        }
    }
}

//struct HitSceneBody
//{
//    HitSceneBody(
//        const OctreeNode* octree, const Ray& ray, float tMin, float tMax)
//        : m_triangles(triangles)
//        , m_tMin(tMin)
//        , m_tMax(tMax)
//        , m_hitMinT(tMax)
//        , m_hitId(-1)
//        , m_ray(&ray) {}
//
//    void operator()(const tbb::blocked_range<int>& range)
//    {
//        const Ray* ray = m_ray;
//        //const Triangle* triangles = m_triangles;
//
//        for (int i = range.begin(); i != range.end(); ++i)
//        {
//            Hit hit;
//            if (HitTriangle(*ray, triangles[i], m_tMin, m_tMax, hit))
//            {
//                if (hit.t < m_hitMinT)
//                {
//                    m_hitMinT = hit.t;
//                    m_hitId = i;
//                    m_outHit = hit;
//                }
//            }
//        }
//    }
//
//    HitSceneBody(const HitSceneBody& hitScene, tbb::split)
//        : m_octree(hitScene.m_octree)
//        , m_ray(hitScene.m_ray)
//        , m_tMin(hitScene.m_tMin)
//        , m_tMax(hitScene.m_tMax)
//        , m_hitId(-1)
//        , m_hitMinT(hitScene.m_tMax) {}
//
//    void join(const HitSceneBody& hitScene)
//    {
//        if (hitScene.m_hitMinT < m_hitMinT)
//        {
//            m_hitMinT = hitScene.m_hitMinT;
//            m_hitId = hitScene.m_hitId;
//            m_outHit = hitScene.m_outHit;
//        }
//    }
//
//    int m_hitId = -1;
//    float m_tMin = 0.0f;
//    float m_tMax = 0.0f;
//    float m_hitMinT;
//    const Ray* m_ray;
//    const OctreeNode* m_octree = nullptr;
//    Hit m_outHit;
//};

void HitSceneInternal(
    const Ray& ray, const OctreeNode& octree, float tMin, float tMax,
    Hit& outHit, int& hitId, float& hitMinT)
{
    float tmin;
    glm::vec3 boxhit;
    if (RayIntersectAabb(ray, octree.m_aabb, tmin, boxhit))
    {
        if (octree.Leaf())
        {
            for (const auto& triangle : octree.m_triangles)
            {
                Hit hit;
                if (HitTriangle(ray, triangle, tMin, tMax, hit))
                {
                    if (hit.t < hitMinT)
                    {
                        hitMinT = hit.t;
                        hitId = 1;
                        outHit = hit;
                    }
                }
            }
        }
        else
        {
            for (const auto& oct : octree.m_children)
            {
                HitSceneInternal(ray, *oct, tMin, tMax, outHit, hitId, hitMinT);
            }
        }
    }
}

// Check all the triangles in the scene for a hit, and return the closest one.
int HitScene(
    const Ray& ray, const OctreeNode& octree, float tMin, float tMax, Hit& outHit)
{
    int hitId = -1;
    float hitMinT = tMax;

    HitSceneInternal(ray, octree, tMin, tMax, outHit, hitId, hitMinT);

    return hitId;

//    auto hitBody = HitSceneBody(
//        scene.m_octree, r, tMin, tMax);
//
//    const uint32_t grainSize = 10000; // disable threading
//    tbb::parallel_reduce(tbb::blocked_range<int>(
//        0, (int)scene.m_triangles.size(), grainSize), hitBody);
//
//    outHit = hitBody.m_outHit;
//    return hitBody.m_hitId;
}
