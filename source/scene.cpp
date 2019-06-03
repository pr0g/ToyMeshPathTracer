#include "scene.h"
#include <string>

#include "tbb/parallel_reduce.h"
#include "tbb/blocked_range.h"

struct OctreeNode
{
    Aabb m_aabb;
    
    std::vector<std::unique_ptr<OctreeNode>> m_children;
    Triangles m_triangles;
    
    bool Leaf() const { return m_children.empty(); }
    void Subdivide(int depth = 0);

private:
    void InternalDivide(int depth);
};

static void HitSceneInternal(
    const Ray& ray, const Ray& invRay, const OctreeNode& octree,
    float tMin, float tMax, Hit& outHit, int& hitId, float& hitMinT)
{
    if (RayHitAabb(invRay, octree.m_aabb, tMin, tMax))
    {
        if (octree.Leaf())
        {
            for (size_t i = 0; i < octree.m_triangles.count(); ++i)
            {
                Hit hit;
                if (RayIntersectTriangleImproved(ray, octree.m_triangles.tri(i), tMin, tMax, hit))
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
                HitSceneInternal(
                    ray, invRay, *oct, tMin, tMax, outHit, hitId, hitMinT);
            }
        }
    }
}

Scene::Scene(const Triangles& triangles)
{
    m_triangles.v0 = triangles.v0;
    m_triangles.v1 = triangles.v1;
    m_triangles.v2 = triangles.v2;
}

Scene::~Scene() = default;

void Scene::BuildOctree(const glm::vec3& min, const glm::vec3& max)
{
    m_octree = std::make_unique<OctreeNode>();
    
    m_octree->m_aabb.min = min;
    m_octree->m_aabb.max = max;
    m_octree->m_triangles = m_triangles;
    m_octree->Subdivide();
}

// Check all the triangles in the scene for a hit, and return the closest one.
int Scene::HitScene(
    const Ray& ray, float tMin, float tMax, Hit& outHit) const
{
    int hitId = -1;
    float hitMinT = tMax;

    Ray invR = ray;
    invR.dir = glm::vec3(1.0f) / ray.dir;
    HitSceneInternal(ray, invR, *m_octree, tMin, tMax, outHit, hitId, hitMinT);

    return hitId;
}

void OctreeNode::Subdivide(int depth)
{
    if (m_triangles.count() > 10 && depth < 10)
    {
        InternalDivide(depth + 1);
    }
}

void OctreeNode::InternalDivide(int depth)
{
    const glm::vec3 halfDimensions = m_aabb.dimensions() * 0.5f;

    // allocate children
    m_children.resize(8);
    for (int64_t i = 0; i < 8; ++i)
    {
        m_children[i] = std::make_unique<OctreeNode>();
    }

    // initialize dimensions
    m_children[0]->m_aabb.min = m_aabb.min;
    m_children[0]->m_aabb.max = m_children[0]->m_aabb.min + halfDimensions;

    m_children[1]->m_aabb.min = m_aabb.min + glm::vec3(halfDimensions.x, 0.0f, 0.0f);
    m_children[1]->m_aabb.max = m_children[1]->m_aabb.min + halfDimensions;

    m_children[2]->m_aabb.min = m_aabb.min + glm::vec3(0.0f, 0.0f, halfDimensions.z);
    m_children[2]->m_aabb.max = m_children[2]->m_aabb.min + halfDimensions;

    m_children[3]->m_aabb.min = m_aabb.min + glm::vec3(halfDimensions.x, 0.0f, halfDimensions.z);
    m_children[3]->m_aabb.max = m_children[3]->m_aabb.min + halfDimensions;

    m_children[4]->m_aabb.min = m_aabb.min + glm::vec3(0.0f, halfDimensions.y, 0.0f);
    m_children[4]->m_aabb.max = m_children[4]->m_aabb.min + halfDimensions;

    m_children[5]->m_aabb.min = m_aabb.min + glm::vec3(halfDimensions.x, halfDimensions.y, 0.0f);
    m_children[5]->m_aabb.max = m_children[5]->m_aabb.min + halfDimensions;

    m_children[6]->m_aabb.min = m_aabb.min + glm::vec3(0.0f, halfDimensions.y, halfDimensions.z);
    m_children[6]->m_aabb.max = m_children[6]->m_aabb.min + halfDimensions;

    m_children[7]->m_aabb.min = m_aabb.min + halfDimensions;
    m_children[7]->m_aabb.max = m_children[7]->m_aabb.min + halfDimensions;

    // iterate children and redistribute
    for (const auto& node : m_children)
    {
        const auto center = node->m_aabb.center();
        const auto halfDimensions = node->m_aabb.dimensions() * 0.5f;
        for (size_t i = 0; i < m_triangles.count(); ++i)
        {
            const Triangle& tri = m_triangles.tri(i);
            if (TriangleIntersectAabb(center, halfDimensions, tri))
            {
                node->m_triangles.add(tri);
            }
        }

        node->Subdivide(depth);
    }

    m_triangles.clear();
}
