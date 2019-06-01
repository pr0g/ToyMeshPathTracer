#pragma once

// Scene: this represents all the scene geometry that the ray tracer works on.

#include <vector>
#include <memory>
#include <algorithm>

#include "tbb/cache_aligned_allocator.h"

#include "maths.h"

struct OctreeNode
{
    Aabb m_aabb;
    std::vector<std::unique_ptr<OctreeNode>> m_children;
    std::vector<Triangle, tbb::cache_aligned_allocator<Triangle>> m_triangles;
    bool Leaf() const { return m_children.empty(); }

    void InternalDivide(int depth)
    {
        glm::vec3 halfDimensions = m_aabb.dimensions() * 0.5f;

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

    void Subdivide(int depth)
    {
        if (m_triangles.size() > 10 && depth < 10)
        {
            InternalDivide(depth + 1);
        }
    }
};

// Our scene structure is very simple: just a bunch of triangles and nothing else
// (no "objects", "instances" or "materials").
struct Scene
{
    Scene(const Triangle* triangles, int triangleCount);

    // doesn't work for ray tracing as triangles in shadow
    // are incorrectly discarded (shadows appear broken)
    void Cull(const glm::vec3& lookFrom);

    void BuildOctree(const glm::vec3& min, const glm::vec3& max);

    OctreeNode m_octree;

private:
    // Scene information: just a copy of the input triangles.
    std::vector<Triangle, tbb::cache_aligned_allocator<Triangle>> m_triangles;
};

// Checks if the ray segment hits a scene. If any triangle is hit by the ray, this
// function should return information about the closest one.
//
// - r: the ray itself,
// - tMin and tMax: segment of the ray that is checked,
// - outHit: hit information, if any,
//
// Function returns the triangle index, or -1 if nothing is hit by the ray.
int HitScene(
    const Ray& r, const OctreeNode& octree, float tMin, float tMax, Hit& outHit);
