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

rtm::vector4f RandomInUnitDisk(uint32_t& state)
{
    rtm::vector4f p;
    do
    {
        p = rtm::vector_sub(
                rtm::vector_mul(
                    rtm::vector_set(RandomFloat01(state),RandomFloat01(state),0.0f),
                    2.0f),
                rtm::vector_set(1.0f,1.0f,0.0f));
    } while (rtm::vector_dot3(p, p) >= 1.0f);
    return p;
}

rtm::vector4f RandomUnitVector(uint32_t& state)
{
    float z = RandomFloat01(state) * 2.0f - 1.0f;
    float a = RandomFloat01(state) * 2.0f * kPI;
    float r = sqrtf(1.0f - z * z);
    float x = r * cosf(a);
    float y = r * sinf(a);
    return rtm::vector_set(x, y, z, 0.0f);
}
