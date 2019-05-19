#include "maths.h"
#include <stdlib.h>
#include <stdint.h>

static uint32_t XorShift32(uint32_t& state)
{
    uint32_t x = state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 15;
    state = x;
    return x;
}

float RandomFloat01(uint32_t& state)
{
    return (XorShift32(state) & 0xFFFFFF) / 16777216.0f;
}

as::vec3_t RandomInUnitDisk(uint32_t& state)
{
    as::vec3_t p;
    do
    {
        p = 2.0f * as::vec3_t(RandomFloat01(state),RandomFloat01(state),0.0f) - as::vec3_t(1.0f,1.0f,0.0f);
    } while (as::vec::dot(p,p) >= 1.0f);
    return p;
}

as::vec3_t RandomUnitVector(uint32_t& state)
{
    float z = RandomFloat01(state) * 2.0f - 1.0f;
    float a = RandomFloat01(state) * 2.0f * kPI;
    float r = sqrtf(1.0f - z * z);
    float x = r * cosf(a);
    float y = r * sinf(a);
    return as::vec3_t(x, y, z);
}
