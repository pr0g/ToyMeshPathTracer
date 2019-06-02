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

Camera::Camera(
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

bool RayIntersectAabb(
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

bool RayIntersectTriangle(
    const Ray& r, const Triangle& tri, float tMin, float tMax, Hit& outHit)
{
    glm::vec3 edge0 = tri.v1 - tri.v0;
    glm::vec3 edge1 = tri.v2 - tri.v1;
    glm::vec3 normal = normalize(cross(edge0, edge1));
    float planeOffset = dot(tri.v0, normal);

    glm::vec3 p0 = r.pointAt(tMin);
    glm::vec3 p1 = r.pointAt(tMax);

    float offset0 = dot(p0, normal);
    float offset1 = dot(p1, normal);

    // does the ray segment between tMin & tMax intersect the triangle plane?
    if ((offset0 - planeOffset) * (offset1 - planeOffset) <= 0.0f)
    {
        float t = tMin + (tMax - tMin)*(planeOffset - offset0) / (offset1 - offset0);
        glm::vec3 p = r.pointAt(t);

        glm::vec3 c0 = cross(edge0, p - tri.v0);
        glm::vec3 c1 = cross(edge1, p - tri.v1);
        if (dot(c0, c1) >= 0.f)
        {
            auto edge2 = tri.v0 - tri.v2;
            auto c2 = cross(edge2, p - tri.v2);
            if (dot(c1, c2) >= 0.0f)
            {
                outHit.t = t;
                outHit.pos = p;
                outHit.normal = normal;
                return true;
            }
        }
    }

    return false;
}

constexpr float Epsilon = 1e-5f;
bool RayIntersectTriangleImproved(
    const Ray& r, const Triangle& tri, float tMin, float tMax, Hit& outHit)
{
    const glm::vec3 edge1 = tri.v1 - tri.v0;
    const glm::vec3 edge2 = tri.v2 - tri.v0;

    glm::vec3 pvec = glm::cross(r.dir, edge2);
    const float det = glm::dot(edge1, pvec);

    if (det > -Epsilon && det < Epsilon)
    {
        return false;
    }

    const float invDet = 1.0f / det;

    const glm::vec3 tvec = r.orig - tri.v0;
    float u = glm::dot(tvec, pvec) * invDet;
    if (u < 0.0f || u > 1.0f)
    {
        return false;
    }

    const glm::vec3 qvec = glm::cross(tvec, edge1);
    float v = glm::dot(r.dir, qvec) * invDet;
    if (v < 0.0f || u + v > 1.0f)
    {
        return false;
    }

    const float t = glm::dot(edge2, qvec) * invDet;
    if (t >= tMin && t <= tMax)
    {
        outHit.t = t;
        outHit.pos = (1.0f - u - v) * tri.v0 + u * tri.v1 + v * tri.v2;
        outHit.normal = glm::normalize(glm::cross(edge1, edge2));
        return true;
    }

    return false;
}
