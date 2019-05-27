#pragma once

// Scene: this represents all the scene geometry that the ray tracer works on.

#include <vector>
#include <memory>
#include <algorithm>

#include "tbb/cache_aligned_allocator.h"

#include "maths.h"

// One triangle: just three vertex positions.
struct Triangle
{
    glm::vec3 v0, v1, v2;
};

// Our scene structure is very simple: just a bunch of triangles and nothing else
// (no "objects", "instances" or "materials").
struct Scene
{
    Scene(const Triangle* triangles, int triangleCount);

    // doesn't work for ray tracing as triangles in shadow
    // are incorrectly discarded (shadows appear broken)
    void Cull(const glm::vec3& lookFrom);

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
    const Ray& r, const Scene& scene, float tMin, float tMax, Hit& outHit);
