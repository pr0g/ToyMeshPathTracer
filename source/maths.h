#pragma once

// --------------------------------------------------------------------------
// various math utilities

#define NOMINMAX
#include <cmath>
#include <assert.h>
#include <stdint.h>

#include "glm/glm.hpp"

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
