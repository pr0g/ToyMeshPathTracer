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

/*======================== X-tests ========================*/

#define AXISTEST_X01(a, b, fa, fb)                      \
    p0 = a*v0[1] - b*v0[2];                             \
    p2 = a*v2[1] - b*v2[2];                             \
    if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;}  \
    rad = fa * boxhalfsize[1] + fb * boxhalfsize[2];    \
    if(min>rad || max<-rad) return 0;

#define AXISTEST_X2(a, b, fa, fb)                       \
    p0 = a*v0[1] - b*v0[2];                             \
    p1 = a*v1[1] - b*v1[2];                             \
    if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;}  \
    rad = fa * boxhalfsize[1] + fb * boxhalfsize[2];    \
    if(min>rad || max<-rad) return 0;

/*======================== Y-tests ========================*/

#define AXISTEST_Y02(a, b, fa, fb)                      \
    p0 = -a*v0[0] + b*v0[2];                            \
    p2 = -a*v2[0] + b*v2[2];                            \
    if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;}  \
    rad = fa * boxhalfsize[0] + fb * boxhalfsize[2];    \
    if(min>rad || max<-rad) return 0;

#define AXISTEST_Y1(a, b, fa, fb)                       \
    p0 = -a*v0[0] + b*v0[2];                            \
    p1 = -a*v1[0] + b*v1[2];                            \
    if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;}  \
    rad = fa * boxhalfsize[0] + fb * boxhalfsize[2];    \
    if(min>rad || max<-rad) return 0;

/*======================== Z-tests ========================*/

#define AXISTEST_Z12(a, b, fa, fb)                      \
    p1 = a*v1[0] - b*v1[1];                             \
    p2 = a*v2[0] - b*v2[1];                             \
    if(p2<p1) {min=p2; max=p1;} else {min=p1; max=p2;}  \
    rad = fa * boxhalfsize[0] + fb * boxhalfsize[1];    \
    if(min>rad || max<-rad) return 0;

#define AXISTEST_Z0(a, b, fa, fb)                       \
    p0 = a*v0[0] - b*v0[1];                             \
    p1 = a*v1[0] - b*v1[1];                             \
    if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;}  \
    rad = fa * boxhalfsize[0] + fb * boxhalfsize[1];    \
    if(min>rad || max<-rad) return 0;

#define FINDMINMAX(x0,x1,x2,min,max)                    \
  min = max = x0;                                       \
  if(x1<min) min=x1;                                    \
  if(x1>max) max=x1;                                    \
  if(x2<min) min=x2;                                    \
  if(x2>max) max=x2;

bool PlaneIntersectAabb(
    const glm::vec3& normal, const glm::vec3& vert, const glm::vec3& maxbox)
{
    float v;
    glm::vec3 vmin, vmax;
    for (int q = 0; q <= 2; q++)
    {
        v = vert[q];

        if (normal[q] > 0.0f)
        {
            vmin[q] = -maxbox[q] - v;
            vmax[q] =  maxbox[q] - v;
        }
        else
        {
            vmin[q] =  maxbox[q] - v;
            vmax[q] = -maxbox[q] - v;
        }
    }

    if (glm::dot(normal, vmin) > 0.0f)
    {
        return false;
    }

    if (glm::dot(normal, vmax) >= 0.0f)
    {
        return true;
    }

    return false;
}

bool TriangleIntersectAabb(
    const glm::vec3& boxcenter, const glm::vec3& boxhalfsize, const Triangle& triangle)
{
    /*    use separating axis theorem to test overlap between triangle and box */
    /*    need to test for overlap in these directions: */
    /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
    /*       we do not even need to test these) */
    /*    2) normal of the triangle */
    /*    3) crossproduct(edge from tri, {x,y,z}-directin) */
    /*       this gives 3x3=9 more tests */

    float min, max, p0, p1, p2, rad, fex, fey, fez;

    /* This is the fastest branch on Sun */
    /* move everything so that the boxcenter is in (0,0,0) */

    //SUB(v0,triverts[0],boxcenter);
    glm::vec3 v0 = triangle.v0 - boxcenter;
    //SUB(v1,triverts[1],boxcenter);
    glm::vec3 v1 = triangle.v1 - boxcenter;
    //SUB(v2,triverts[2],boxcenter);
    glm::vec3 v2 = triangle.v2 - boxcenter;

    /* compute triangle edges */

    //SUB(e0,v1,v0);      /* tri edge 0 */
    glm::vec3 e0 = v1 - v0;
    //SUB(e1,v2,v1);      /* tri edge 1 */
    glm::vec3 e1 = v2 - v1;
    //SUB(e2,v0,v2);      /* tri edge 2 */
    glm::vec3 e2 = v0 - v2;

    /* Bullet 3:  */
    /*  test the 9 tests first (this was faster) */

    fex = fabsf(e0[0]);
    fey = fabsf(e0[1]);
    fez = fabsf(e0[2]);

    AXISTEST_X01(e0[2], e0[1], fez, fey);
    AXISTEST_Y02(e0[2], e0[0], fez, fex);
    AXISTEST_Z12(e0[1], e0[0], fey, fex);

    fex = fabsf(e1[0]);
    fey = fabsf(e1[1]);
    fez = fabsf(e1[2]);

    AXISTEST_X01(e1[2], e1[1], fez, fey);
    AXISTEST_Y02(e1[2], e1[0], fez, fex);
    AXISTEST_Z0(e1[1], e1[0], fey, fex);

    fex = fabsf(e2[0]);
    fey = fabsf(e2[1]);
    fez = fabsf(e2[2]);

    AXISTEST_X2(e2[2], e2[1], fez, fey);
    AXISTEST_Y1(e2[2], e2[0], fez, fex);
    AXISTEST_Z12(e2[1], e2[0], fey, fex);

    /* Bullet 1: */
    /*  first test overlap in the {x,y,z}-directions */
    /*  find min, max of the triangle each direction, and test for overlap in */
    /*  that direction -- this is equivalent to testing a minimal AABB around */
    /*  the triangle against the AABB */

    /* test in X-direction */
    FINDMINMAX(v0[0], v1[0], v2[0],min, max);
    if (min > boxhalfsize[0] || max < -boxhalfsize[0])
    {
        return false;
    }

    /* test in Y-direction */
    FINDMINMAX(v0[1], v1[1], v2[1], min, max);
    if (min > boxhalfsize[1] || max < -boxhalfsize[1])
    {
        return false;
    }

    /* test in Z-direction */
    FINDMINMAX(v0[2], v1[2], v2[2], min, max);
    if (min > boxhalfsize[2] || max < -boxhalfsize[2])
    {
        return false;
    }

   /* Bullet 2: */
   /*  test if the box intersects the plane of the triangle */
   /*  compute plane equation of triangle: normal*x+d=0 */

    //CROSS(normal,e0,e1);
    glm::vec3 normal = glm::cross(e0, e1);
    if (!PlaneIntersectAabb(normal, v0, boxhalfsize))
    {
        return false;
    }

    // box and triangle overlaps
    return true;
}
