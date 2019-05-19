#pragma once

// --------------------------------------------------------------------------
// various math utilities

#define NOMINMAX
#include <cmath>
#include <assert.h>
#include <stdint.h>

#include "rtm/vector4f.h"

#define kPI 3.1415926f


// --------------------------------------------------------------------------
// simple 3D vector with x,y,z components

//struct float3
//{
//    float3() : x(0), y(0), z(0) {}
//    float3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
//
//    float3 operator-() const { return float3(-x, -y, -z); }
//    float3& operator+=(const float3& o) { x+=o.x; y+=o.y; z+=o.z; return *this; }
//    float3& operator-=(const float3& o) { x-=o.x; y-=o.y; z-=o.z; return *this; }
//    float3& operator*=(const float3& o) { x*=o.x; y*=o.y; z*=o.z; return *this; }
//    float3& operator*=(float o) { x*=o; y*=o; z*=o; return *this; }
//
//    inline float getX() const { return x; }
//    inline float getY() const { return y; }
//    inline float getZ() const { return z; }
//    inline void setX(float x_) { x = x_; }
//    inline void setY(float y_) { y = y_; }
//    inline void setZ(float z_) { z = z_; }
//    inline void store(float *p) const { p[0] = getX(); p[1] = getY(); p[2] = getZ(); }
//
//    float x, y, z;
//};

//inline float3 operator+(const float3& a, const float3& b) { return float3(a.x+b.x,a.y+b.y,a.z+b.z); }
//inline float3 operator-(const float3& a, const float3& b) { return float3(a.x-b.x,a.y-b.y,a.z-b.z); }
//inline float3 operator*(const float3& a, const float3& b) { return float3(a.x*b.x,a.y*b.y,a.z*b.z); }
//inline float3 operator*(const float3& a, float b) { return float3(a.x*b,a.y*b,a.z*b); }
//inline float3 operator*(float a, const float3& b) { return float3(a*b.x,a*b.y,a*b.z); }
//
//inline float dot(const float3& a, const float3& b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
//
//inline float3 cross(const float3& a, const float3& b)
//{
//    return float3(a.y*b.z - a.z*b.y, -(a.x*b.z - a.z*b.x), a.x*b.y - a.y*b.x);
//}
//
//inline float3 min(const float3& a, const float3& b)
//{
//    return float3(fmin(a.x,b.x), fmin(a.y,b.y), fmin(a.z,b.z));
//}
//inline float3 max(const float3& a, const float3& b)
//{
//    return float3(fmax(a.x,b.x), fmax(a.y,b.y), fmax(a.z,b.z));
//}
//
//inline float length(float3 v) { return sqrtf(dot(v, v)); }
//inline float sqLength(float3 v) { return dot(v, v); }
//inline float3 normalize(float3 v) { return v * (1.0f / length(v)); }
//

inline float saturate(float v) { if (v < 0.0f) return 0.0f; if (v > 1.0f) return 1.0f; return v; }

inline void AssertUnit(rtm::vector4f_arg0 v)
{
    (void)v;
    assert(fabsf(rtm::vector_length_squared(v) - 1.0f) < 0.01f);
}


// --------------------------------------------------------------------------
// ray: starting position (origin) and direction.
// direction is assumed to be normalized

struct Ray
{
    Ray() {}
    Ray(rtm::vector4f_arg0 orig_, rtm::vector4f_arg1 dir_) : orig(orig_), dir(dir_) { AssertUnit(dir); }

    rtm::vector4f pointAt(float t) const { return rtm::vector_add(orig, rtm::vector_mul(dir, t)); }

    rtm::vector4f orig;
    rtm::vector4f dir;
};


// --------------------------------------------------------------------------
// ray hit point information: position where it hit something;
// normal of the surface that was hit, and "t" position along the ray

struct Hit
{
    rtm::vector4f pos;
    rtm::vector4f normal;
    float t;
};


// --------------------------------------------------------------------------
// random number generator utilities

float RandomFloat01(uint32_t& state);
rtm::vector4f RandomInUnitDisk(uint32_t& state);
rtm::vector4f RandomUnitVector(uint32_t& state);


// --------------------------------------------------------------------------
// camera

struct Camera
{
    Camera() {}

    // vfov is top to bottom in degrees
    Camera(
        rtm::vector4f_arg0 lookFrom, rtm::vector4f_arg1 lookAt, rtm::vector4f_arg2 vup,
        float vfov, float aspect, float aperture, float focusDist)
    {
        lensRadius = aperture * 0.5f;
        float theta = vfov * kPI / 180.0f;
        float halfHeight = tanf(theta* 0.5f);
        float halfWidth = aspect * halfHeight;
        origin = lookFrom;
        w = rtm::vector_normalize3(rtm::vector_sub(lookFrom, lookAt), rtm::vector_set(0.0f));
        u = rtm::vector_normalize3(rtm::vector_cross3(vup, w), rtm::vector_set(0.0f));
        v = rtm::vector_cross3(w, u);
        lowerLeftCorner = origin - halfWidth*focusDist*u - halfHeight*focusDist*v - focusDist*w;
        horizontal = 2.0f * halfWidth * focusDist * u;
        vertical = 2.0f * halfHeight * focusDist * v;
    }

    Ray GetRay(float s, float t, uint32_t& state) const
    {
        rtm::vector4f rd = lensRadius * RandomInUnitDisk(state);
        rtm::vector4f offset =
            rtm::vector_add(
                rtm::vector_mul(
                    u, rtm::vector_get_x(rd)),
                rtm::vector_mul(
                    v, rtm::vector_get_y(rd)));
        return Ray(
            rtm::vector_add(origin, offset),
            rtm::vector_normalize3(
                rtm::vector_add(lowerLeftCorner,
                    rtm::vector_sub(
                        rtm::vector_add(rtm::vector_mul(horizontal, s), rtm::vector_mul(vertical, t)),
                        rtm::vector_add(origin, offset))), rtm::vector_set(0.0f)));
    }

    rtm::vector4f origin;
    rtm::vector4f lowerLeftCorner;
    rtm::vector4f horizontal;
    rtm::vector4f vertical;
    rtm::vector4f u, v, w;
    float lensRadius;
};
