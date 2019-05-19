#pragma once

// --------------------------------------------------------------------------
// various math utilities

#define NOMINMAX
#include <cmath>
#include <assert.h>
#include <stdint.h>

//#include "glm/glm.hpp"
#include "as/math/as-vec.hpp"
#include "as/math/as-math-ops.hpp"
#include "as/math/as-math.hpp"

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
        const as::vec3_t& lookFrom, const as::vec3_t& lookAt, const as::vec3_t& vup,
        float vfov, float aspect, float aperture, float focusDist)
    {
        lensRadius = aperture * 0.5f;
        float theta = vfov * kPI / 180.0f;
        float halfHeight = tanf(theta * 0.5f);
        float halfWidth = aspect * halfHeight;
        origin = lookFrom;
        w = as::vec::normalize(lookFrom - lookAt);
        u = as::vec::normalize(as::vec3::cross(vup, w));
        v = as::vec3::cross(w, u);
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
