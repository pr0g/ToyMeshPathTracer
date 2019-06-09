#pragma once

// Scene: this represents all the scene geometry that the ray tracer works on.

#include <vector>
#include <memory>
#include <algorithm>

#include "tbb/cache_aligned_allocator.h"

#include "maths.h"

struct OctreeNode;

// Our scene structure is very simple: just a bunch of triangles and nothing else
// (no "objects", "instances" or "materials").
struct Scene
{
    Scene(const Triangles& triangles);
    ~Scene();

    void BuildOctree(const as::vec3_t& min, const as::vec3_t& max);
    
    // Checks if the ray segment hits a scene. If any triangle is hit by the ray, this
    // function should return information about the closest one.
    //
    // - r: the ray itself,
    // - tMin and tMax: segment of the ray that is checked,
    // - outHit: hit information, if any,
    //
    // Function returns the triangle index, or -1 if nothing is hit by the ray.
    int HitScene(
        const Ray& r, float tMin, float tMax, Hit& outHit) const;

private:
    // Scene information: just a copy of the input triangles.
    Triangles m_triangles;
    std::unique_ptr<OctreeNode> m_octree;
};
