#pragma once

// --------------------------------------------------------------------------
// various math utilities

#define NOMINMAX
#include <cmath>
#include <assert.h>
#include <stdint.h>
#include <vector>

#include "external/as/math/as-vec.hpp"
#include "external/as/math/as-math-ops.hpp"

#include "tbb/cache_aligned_allocator.h"

#define kPI 3.1415926f

inline float saturate(float v)
{
    return as::clamp(v, 0.0f, 1.0f);
}

inline void AssertUnit(const as::vec3_t& v)
{
    (void)v;
    assert(fabsf(as::vec::length(v) - 1.0f) < 0.01f);
}

// --------------------------------------------------------------------------
// ray: starting position (origin) and direction.
// direction is assumed to be normalized

struct Ray
{
    Ray() {}
    Ray(const as::vec3_t& orig_, const as::vec3_t& dir_) : orig(orig_), dir(dir_) { AssertUnit(dir); }

    as::vec3_t pointAt(float t) const { return orig + dir * t; }

    as::vec3_t orig;
    as::vec3_t dir;
};

// --------------------------------------------------------------------------
// ray hit point information: position where it hit something;
// normal of the surface that was hit, and "t" position along the ray

struct Hit
{
    as::vec3_t pos;
    as::vec3_t normal;
    float t;
};

// --------------------------------------------------------------------------
// one triangle: three vertex positions.

struct Triangle
{
    as::vec3_t v0, v1, v2;
};

struct Triangles
{
    std::vector<as::vec3_t, tbb::cache_aligned_allocator<as::vec3_t>> v0;
    std::vector<as::vec3_t, tbb::cache_aligned_allocator<as::vec3_t>> v1;
    std::vector<as::vec3_t, tbb::cache_aligned_allocator<as::vec3_t>> v2;
    
    void add(const Triangle& triangle)
    {
        v0.push_back(triangle.v0);
        v1.push_back(triangle.v1);
        v2.push_back(triangle.v2);
    }
    
    void clear()
    {
        v0.clear();
        v1.clear();
        v2.clear();
    }
    
    size_t count() const { return v0.size(); }
    
    Triangle tri(const size_t i) const
    {
        return Triangle { v0[i], v1[i], v2[i] };
    }
};

// --------------------------------------------------------------------------
// aabb: stored as min and max extents

struct Aabb
{
    as::vec3_t min;
    as::vec3_t max;

    as::vec3_t center() const { return (min + max) * 0.5f; }
    as::vec3_t dimensions() const { return max - min; }
};

// --------------------------------------------------------------------------
// random number generator utilities

float RandomFloat01(uint32_t& state);
as::vec3_t RandomInUnitDisk(uint32_t& state);
as::vec3_t RandomUnitVector(uint32_t& state);

// --------------------------------------------------------------------------
// camera

struct Camera
{
    Camera() = default;

    // vfov is top to bottom in degrees
    Camera(
        const as::vec3_t& lookFrom, const as::vec3_t& lookAt,
        const as::vec3_t& vup, float vfov, float aspect,
        float aperture, float focusDist);

    Ray GetRay(float s, float t, uint32_t& state) const
    {
        as::vec3_t rd = lensRadius * RandomInUnitDisk(state);
        as::vec3_t offset = u * rd.x + v * rd.y;
        return Ray(
            origin + offset,
            as::vec::normalize(
                lowerLeftCorner +
                s * horizontal +
                t * vertical -
                origin - offset));
    }

    as::vec3_t origin;
    as::vec3_t lowerLeftCorner;
    as::vec3_t horizontal;
    as::vec3_t vertical;
    as::vec3_t u, v, w;
    float lensRadius;
};

// from Peter Shirley's "Ray Tracing: The Next Week"
// note: ray direction should be inverted, i.e 1.0/direction!
inline bool RayHitAabb(
    const Ray& r, const Aabb& box, float tMin, float tMax)
{
#define DO_COORD(c) \
    { \
        float t0 = (box.min.c - r.orig.c) * r.dir.c; \
        float t1 = (box.max.c - r.orig.c) * r.dir.c; \
        if (r.dir.c < 0.0f) \
            std::swap(t0, t1); \
        tMin = t0 > tMin ? t0 : tMin; \
        tMax = t1 < tMax ? t1 : tMax; \
        if (tMax < tMin) \
            return false; \
    }
    DO_COORD(x);
    DO_COORD(y);
    DO_COORD(z);
    return true;
}

// Real-Time Collision Detection - Christer Ericson
// Section 5.3.3
// Intersect ray R(t) = p + t*d against AABB a. When intersecting,
// return intersection distance tmin and point q of intersection
bool RayIntersectAabb(
    const Ray& ray, const Aabb& aabb, float &tmin, as::vec3_t& q);

// Checks if one triangle is hit by a ray segment.
bool RayIntersectTriangle(
    const Ray& r, const Triangle& tri, float tMin, float tMax, Hit& outHit);

// Fast Minimum Storage RayTriangle Intersection - Tomas Moller, Ben Trumb
// https://cadxfem.org/inf/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
bool RayIntersectTriangleImproved(
    const Ray& r, const Triangle& tri, float tMin, float tMax, Hit& outHit);

// Same as above but accepting triangles in soa format
bool RayIntersectTrianglesImproved(
    const Ray& r, const Triangles& tris, int64_t index,
    float tMin, float tMax, Hit& outHit);

// Reference Fast 3D Triangle-Box Overlap Testing by Tomas Akenine-Moller
// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/pubs/tribox.pdf
// http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tribox3.txt
bool PlaneIntersectAabb(
    const as::vec3_t& normal, const as::vec3_t& vert, const as::vec3_t& maxbox);

// Reference Fast 3D Triangle-Box Overlap Testing by Tomas Akenine-Moller
// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/pubs/tribox.pdf
// http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tribox3.txt
bool TriangleIntersectAabb(
    const as::vec3_t& boxcenter, const as::vec3_t& boxhalfsize,
    const Triangle& triangle);
