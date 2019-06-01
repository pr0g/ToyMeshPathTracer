#pragma once

// --------------------------------------------------------------------------
// various math utilities

#define NOMINMAX
#include <cmath>
#include <assert.h>
#include <stdint.h>

#include "glm/glm.hpp"
#include "glm/gtc/epsilon.hpp"

#define kPI 3.1415926f

inline float saturate(float v)
{
    return glm::clamp(v, 0.0f, 1.0f);
}

inline void AssertUnit(const glm::vec3& v)
{
    (void)v;
    assert(fabsf(glm::length(v) - 1.0f) < 0.01f);
}

// --------------------------------------------------------------------------
// ray: starting position (origin) and direction.
// direction is assumed to be normalized

struct Ray
{
    Ray() {}
    Ray(const glm::vec3& orig_, const glm::vec3& dir_) : orig(orig_), dir(dir_) { AssertUnit(dir); }

    glm::vec3 pointAt(float t) const { return orig + dir * t; }

    glm::vec3 orig;
    glm::vec3 dir;
};

// --------------------------------------------------------------------------
// ray hit point information: position where it hit something;
// normal of the surface that was hit, and "t" position along the ray

struct Hit
{
    glm::vec3 pos;
    glm::vec3 normal;
    float t;
};

// --------------------------------------------------------------------------
// one triangle: three vertex positions.

struct Triangle
{
    glm::vec3 v0, v1, v2;
};

// --------------------------------------------------------------------------
// aabb: stored as min and max extents

struct Aabb
{
    glm::vec3 min;
    glm::vec3 max;

    glm::vec3 center() const { return (min + max) * 0.5f; }
    glm::vec3 dimensions() const { return max - min; }
};

// --------------------------------------------------------------------------
// random number generator utilities

float RandomFloat01(uint32_t& state);
glm::vec3 RandomInUnitDisk(uint32_t& state);
glm::vec3 RandomUnitVector(uint32_t& state);

// --------------------------------------------------------------------------
// camera

struct Camera
{
    Camera() = default;

    // vfov is top to bottom in degrees
    Camera(
        const glm::vec3& lookFrom, const glm::vec3& lookAt, const glm::vec3& vup,
        float vfov, float aspect, float aperture, float focusDist)
    {
        lensRadius = aperture * 0.5f;
        float theta = vfov * kPI / 180.0f;
        float halfHeight = tanf(theta * 0.5f);
        float halfWidth = aspect * halfHeight;
        origin = lookFrom;
        w = glm::normalize(lookFrom - lookAt);
        u = glm::normalize(cross(vup, w));
        v = cross(w, u);
        lowerLeftCorner =
            origin -
            halfWidth * focusDist * u -
            halfHeight * focusDist * v -
            focusDist * w;
        horizontal = 2.0f * halfWidth * focusDist * u;
        vertical = 2.0f * halfHeight * focusDist * v;
    }

    Ray GetRay(float s, float t, uint32_t& state) const
    {
        glm::vec3 rd = lensRadius * RandomInUnitDisk(state);
        glm::vec3 offset = u * rd.x + v * rd.y;
        return Ray(
            origin + offset,
            glm::normalize(
                lowerLeftCorner +
                s * horizontal +
                t * vertical -
                origin - offset));
    }

    glm::vec3 origin;
    glm::vec3 lowerLeftCorner;
    glm::vec3 horizontal;
    glm::vec3 vertical;
    glm::vec3 u, v, w;
    float lensRadius;
};

// Real-Time Collision Detection - Christer Ericson
// Section 5.3.3
// Intersect ray R(t) = p + t*d against AABB a. When intersecting,
// return intersection distance tmin and point q of intersection
inline bool RayIntersectAabb(
    const Ray& ray, const Aabb& aabb, float &tmin, glm::vec3& q)
{
    tmin = 0.0f;          // set to -FLT_MAX to get first hit on line
    float tmax = FLT_MAX; // set to max distance ray can travel (for segment)

    // For all three slabs
    for (int i = 0; i < 3; i++)
    {
        constexpr float EPSILON = 0.001f;
        if (fabsf(ray.dir[i]) < EPSILON)
        {
            // Ray is parallel to slab. No hit if origin not within slab
            if (ray.orig[i] < aabb.min[i] || ray.orig[i] > aabb.max[i])
            {
                return false;
            }
        }
        else
        {
            // Compute intersection t value of ray with near and far plane of slab
            float ood = 1.0f / ray.dir[i];
            float t1 = (aabb.min[i] - ray.orig[i]) * ood;
            float t2 = (aabb.max[i] - ray.orig[i]) * ood;

            // Make t1 be intersection with near plane, t2 with far plane
            if (t1 > t2)
            {
                std::swap(t1, t2);
            }

            // Compute the intersection of slab intersections intervals
            tmin = glm::max(tmin, t1);
            tmax = glm::min(tmax, t2);

            // Exit with no collision as soon as slab intersection becomes empty
            if (tmin > tmax)
            {
                return false;
            }
        }
    }

    // Ray intersects all 3 slabs. Return point (q) and intersection t value (tmin)
    q = ray.orig + ray.dir * tmin;

    return true;
}

// Reference Fast 3D Triangle-Box Overlap Testing by Tomas Akenine-Moller
// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/pubs/tribox.pdf
// http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tribox3.txt
bool PlaneIntersectAabb(
    const glm::vec3& normal, const glm::vec3& vert, const glm::vec3& maxbox);

// Reference Fast 3D Triangle-Box Overlap Testing by Tomas Akenine-Moller
// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/pubs/tribox.pdf
// http://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/code/tribox3.txt
bool TriangleIntersectAabb(
    const glm::vec3& boxcenter, const glm::vec3& boxhalfsize,
    const Triangle& triangle);
