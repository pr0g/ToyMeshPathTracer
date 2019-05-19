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

glm::vec3 RandomInUnitDisk(uint32_t& state)
{
    glm::vec3 p;
    do
    {
        p = 2.0f * glm::vec3(RandomFloat01(state),RandomFloat01(state),0.0f) - glm::vec3(1.0f,1.0f,0.0f);
    } while (dot(p,p) >= 1.0f);
    return p;
}

glm::vec3 RandomUnitVector(uint32_t& state)
{
    float z = RandomFloat01(state) * 2.0f - 1.0f;
    float a = RandomFloat01(state) * 2.0f * kPI;
    float r = sqrtf(1.0f - z * z);
    float x = r * cosf(a);
    float y = r * sinf(a);
    return glm::vec3(x, y, z);
}
