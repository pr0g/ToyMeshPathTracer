#include "scene.h"
#include <string>

#include "tbb/parallel_reduce.h"
#include "tbb/blocked_range.h"

struct OctreeNode
{
    Aabb m_aabb;
    
    std::vector<std::unique_ptr<OctreeNode>> m_children;
    std::vector<Triangle, tbb::cache_aligned_allocator<Triangle>> m_triangles;
    
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
            for (const auto& triangle : octree.m_triangles)
            {
                Hit hit;
                if (RayIntersectTriangleImproved(ray, triangle, tMin, tMax, hit))
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

Scene::Scene(const Triangle* triangles, int triangleCount)
{
    m_triangles.assign(triangles, triangles + triangleCount);
}

Scene::~Scene() = default;

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
    if (m_triangles.size() > 10 && depth < 10)
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
        for (const Triangle& triangle : m_triangles)
        {
            if (TriangleIntersectAabb(center, halfDimensions, triangle))
            {
                node->m_triangles.push_back(triangle);
            }
        }

        node->Subdivide(depth);
    }

    m_triangles.clear();
}
