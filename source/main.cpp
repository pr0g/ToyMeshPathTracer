// main program entry point and the actual raytracing bits

#include "maths.h"
#include "scene.h"

// Include external libraries:
// - PNG writing
#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "external/stb_image_write.h"
// - time measurement
#define SOKOL_IMPL
#include "external/sokol_time.h"
// - OBJ file loading
#include "external/objparser.h"

#include <vector>

#include "tbb/parallel_for.h"
#include "tbb/task_scheduler_init.h"
#include "tbb/blocked_range.h"
#include "tbb/scalable_allocator.h"
#include "tbb/cache_aligned_allocator.h"

// --------------------------------------------------------------------------
// "ray/path tracing" bits

// general minimum/maximum distances for rays (from "very close to surface but not exacttly on it"
// to "ten million units")
const float kMinT = 0.001f;
const float kMaxT = 1.0e7f;
// maximum raytracing recursion depth, i.e. number of light bounces
const int kMaxDepth = 10;

// we have one hardcoded directional light, with this direction and color
static const glm::vec3 kLightDir = normalize(glm::vec3(-0.7f,1.0f,0.5f));
static const glm::vec3 kLightColor = glm::vec3(0.7f,0.6f,0.5f);

// when a ray "r" has just hit a surface at point "hit", decide what to do about it:
// in our very simple case, we assume the surface is perfectly diffuse, so we'll return:
// - surface albedo ("color") in "attenuation"
// - new random ray for the next light bounce in "scattered"
// - illumination from the directional light in "outLightE"
static Ray Scatter(
    const Ray& r, const Hit& hit, glm::vec3& outAttenuation,
    glm::vec3& outLightE, uint32_t& rngState, int& inoutRayCount)
{
    outLightE = glm::vec3 { 0.0f };

    // model a perfectly diffuse material:

    // make color slightly based on surface normals
    glm::vec3 albedo = glm::vec3(0.7f);
    outAttenuation = albedo;

    // explicit directional light by shooting a shadow ray
    ++inoutRayCount;
    Hit lightHit;
    int id = HitScene(Ray(hit.pos, kLightDir), kMinT, kMaxT, lightHit);
    if (id == -1)
    {
        // ray towards the light did not hit anything in the scene, so
        // that means we are not in shadow: compute illumination from it
        glm::vec3 rdir = r.dir;
        AssertUnit(rdir);
        glm::vec3 nl = dot(hit.normal, rdir) < 0 ? hit.normal : -hit.normal;
        outLightE += albedo * kLightColor * (fmax(0.0f, dot(kLightDir, nl)));
    }

    // random point on unit sphere that is tangent to the hit point
    glm::vec3 target = hit.pos + hit.normal + RandomUnitVector(rngState);
    return Ray(hit.pos, normalize(target - hit.pos));
}

struct IntermediateScatterResult
{
    glm::vec3 light;
    glm::vec3 attenuation;
};

// trace a ray into the scene, and return the final color for it
static glm::vec3 Trace(
    Ray ray, uint32_t& rngState, int& inoutRayCount)
{
    thread_local IntermediateScatterResult scatteredResults[kMaxDepth];

    int depth = 0;
    glm::vec3 color { 0.0f };
    while (depth < kMaxDepth)
    {
        ++inoutRayCount;
        Hit hit;
        int id = HitScene(ray, kMinT, kMaxT, hit);
        // ray hits something in the scene
        if (id != -1)
        {
            ray = Scatter(
                ray, hit, scatteredResults[depth].attenuation,
                scatteredResults[depth].light, rngState, inoutRayCount);

            ++depth;
        }
        else
        {
            // ray does not hit anything: return illumination from the sky (just a simple gradient really)
            float t = 0.5f * (ray.dir.y + 1.0f);
            color = ((1.0f - t) * glm::vec3(1.0f) + t * glm::vec3(0.5f, 0.7f, 1.0f)) * 0.5f;
            break;
        }
    }
    
    for (int i = depth - 1; i >= 0; --i)
    {
        color = scatteredResults[i].light +
            scatteredResults[i].attenuation * color;
    }

    return color;
}

// load scene from an .OBJ file
static bool LoadScene(const char* dataFile, glm::vec3& outBoundsMin, glm::vec3& outBoundsMax)
{
    ObjFile objFile;
    if (!objParseFile(objFile, dataFile))
    {
        printf("ERROR: failed to load .obj file\n");
        return false;
    }

    outBoundsMin = glm::vec3(+1.0e6f, +1.0e6f, +1.0e6f);
    outBoundsMax = glm::vec3(-1.0e6f, -1.0e6f, -1.0e6f);

    int objTriCount = int(objFile.f_size / 9);
    Triangle* tris = new Triangle[objTriCount + 2]; // will add two triangles for the "floor"
    for (int i = 0; i < objTriCount; ++i)
    {
        int idx0 = objFile.f[i * 9 + 0] * 3;
        int idx1 = objFile.f[i * 9 + 3] * 3;
        int idx2 = objFile.f[i * 9 + 6] * 3;
        glm::vec3 v0 = glm::vec3(objFile.v[idx0 + 0], objFile.v[idx0 + 1], objFile.v[idx0 + 2]);
        glm::vec3 v1 = glm::vec3(objFile.v[idx1 + 0], objFile.v[idx1 + 1], objFile.v[idx1 + 2]);
        glm::vec3 v2 = glm::vec3(objFile.v[idx2 + 0], objFile.v[idx2 + 1], objFile.v[idx2 + 2]);
        tris[i].v0 = v0;
        tris[i].v1 = v1;
        tris[i].v2 = v2;
        outBoundsMin = min(outBoundsMin, v0); outBoundsMax = max(outBoundsMax, v0);
        outBoundsMin = min(outBoundsMin, v1); outBoundsMax = max(outBoundsMax, v1);
        outBoundsMin = min(outBoundsMin, v2); outBoundsMax = max(outBoundsMax, v2);
    }

    // add two triangles that are right "under the scene" and covering larger area than the scene
    // itself, to serve as a "floor"
    glm::vec3 size = outBoundsMax - outBoundsMin;
    glm::vec3 extra = size * 0.7f;
    tris[objTriCount+0].v0 = glm::vec3(outBoundsMin.x-extra.x, outBoundsMin.y, outBoundsMin.z-extra.z);
    tris[objTriCount+0].v1 = glm::vec3(outBoundsMin.x-extra.x, outBoundsMin.y, outBoundsMax.z+extra.z);
    tris[objTriCount+0].v2 = glm::vec3(outBoundsMax.x+extra.x, outBoundsMin.y, outBoundsMin.z-extra.z);
    tris[objTriCount+1].v0 = glm::vec3(outBoundsMin.x-extra.x, outBoundsMin.y, outBoundsMax.z+extra.z);
    tris[objTriCount+1].v1 = glm::vec3(outBoundsMax.x+extra.x, outBoundsMin.y, outBoundsMax.z+extra.z);
    tris[objTriCount+1].v2 = glm::vec3(outBoundsMax.x+extra.x, outBoundsMin.y, outBoundsMin.z-extra.z);

    uint64_t t0 = stm_now();
    InitializeScene(objTriCount + 2, tris);
    printf("Initialized scene '%s' (%i tris) in %.3fs\n", dataFile, objTriCount+2, stm_sec(stm_since(t0)));

    delete[] tris;
    return true;
}

struct TraceData
{
    int screenWidth, screenHeight, samplesPerPixel;
    uint8_t* image;
    const Camera* camera;
    std::atomic<int> rayCount;
};

class TraceImageBody {
public:
    TraceImageBody(TraceData* traceData)
        : m_traceData(traceData)
        , m_invWidth(1.0f / traceData->screenWidth)
        , m_invHeight(1.0f / traceData->screenHeight)
        , m_samplesPerPixelRecip(1.0f / static_cast<float>(traceData->samplesPerPixel))
    {
    }

    void operator()(const tbb::blocked_range<int64_t>& range) const {
        TraceData& data = *m_traceData;
        uint8_t* image = data.image;

        const float invWidth = m_invWidth;
        const float invHeight = m_invHeight;
        const float samplesPerPixelRecip = m_samplesPerPixelRecip;

        int rayCount = 0;
        for (int64_t y = range.begin(); y != range.end(); ++y)
        {
            uint32_t rngState = static_cast<uint32_t>(y) * 9781 + 1;
            for (int64_t x = 0; x < data.screenWidth; ++x)
            {
                glm::vec3 col { 0.0f };
                // we'll trace N slightly jittered rays for each pixel, to get anti-aliasing, loop over them here
                for (int64_t s = 0; s < data.samplesPerPixel; s++)
                {
                    // get a ray from camera, and trace it
                    const Ray ray =
                        data.camera->GetRay(
                            (static_cast<float>(x) + RandomFloat01(rngState)) * invWidth,
                            (static_cast<float>(y) + RandomFloat01(rngState)) * invHeight,
                            rngState);

                    col += Trace(ray, rngState, rayCount);
                }

                col *= samplesPerPixelRecip;

                // simplistic "gamma correction" by just taking a square root of the final color
                col.x = sqrtf(col.x);
                col.y = sqrtf(col.y);
                col.z = sqrtf(col.z);

                // our image is bytes in 0-255 range, turn our floats into them here and write into the image
                const int64_t lookup = (y * data.screenWidth + x) * 4;
                image[lookup + 0] = uint8_t(saturate(col.x) * 255.0f);
                image[lookup + 1] = uint8_t(saturate(col.y) * 255.0f);
                image[lookup + 2] = uint8_t(saturate(col.z) * 255.0f);
                image[lookup + 3] = 255;
            }
        }

        data.rayCount += rayCount;
    }

private:
    TraceData* m_traceData = nullptr;
    float m_invWidth;
    float m_invHeight;
    float m_samplesPerPixelRecip;
};

int main(int argc, const char** argv)
{
    const auto threadCount = tbb::task_scheduler_init::default_num_threads();
    tbb::task_scheduler_init init(threadCount);

    // initialize timer
    stm_setup();

    // parse screen size command line arguments
    int screenWidth, screenHeight, samplesPerPixel;
    if (argc < 5)
    {
        printf("Usage: TrimeshTracer.exe [width] [height] [samplesPerPixel] [objFile]\n");
        return 1;
    }
    screenWidth = atoi(argv[1]);
    if (screenWidth < 1 || screenWidth > 10000)
    {
        printf("ERROR: invalid width argument '%s'\n", argv[1]);
        return 1;
    }
    screenHeight = atoi(argv[2]);
    if (screenHeight < 1 || screenHeight > 10000)
    {
        printf("ERROR: invalid height argument '%s'\n", argv[2]);
        return 1;
    }
    samplesPerPixel = atoi(argv[3]);
    if (samplesPerPixel < 1 || samplesPerPixel > 1024)
    {
        printf("ERROR: invalid samplesPerPixel argument '%s'\n", argv[3]);
        return 1;
    }

    // load model file and initialize the scene
    glm::vec3 sceneMin, sceneMax;
    if (!LoadScene("/Users/tomhultonharrop/Documents/Projects/ray-tracing-interview/data/suzanne.obj", sceneMin, sceneMax))
        return 1;

    // place a camera: put it a bit outside scene bounds, looking at the center of it
    glm::vec3 sceneSize = sceneMax - sceneMin;
    glm::vec3 sceneCenter = (sceneMin + sceneMax) * 0.5f;
    glm::vec3 lookfrom = sceneCenter + sceneSize * glm::vec3(0.3f,0.6f,1.2f);
    if (strstr(argv[4], "sponza.obj") != nullptr) // sponza looks bad when viewed from outside; hardcode camera position
        lookfrom = glm::vec3(-5.96f, 4.08f, -1.22f);
    glm::vec3 lookat = sceneCenter + sceneSize * glm::vec3(0.0f, -0.1f, 0.0f);
    const float distToFocus = length(lookfrom - lookat);
    const float aperture = 0.03f;
    auto camera = Camera(
        lookfrom, lookat, glm::vec3(0.0f, 1.0f, 0.0f), 60.0f,
        float(screenWidth) / float(screenHeight), aperture, distToFocus);

    // create RGBA image for the result
    std::vector<uint8_t, tbb::cache_aligned_allocator<uint8_t>> image(
        screenWidth * screenHeight * 4, 0);

    // generate the image - run TraceImage
    uint64_t t0 = stm_now();

    TraceData data;
    data.screenWidth = screenWidth;
    data.screenHeight = screenHeight;
    data.samplesPerPixel = samplesPerPixel;
    data.image = image.data();
    data.camera = &camera;
    data.rayCount = 0;

    const uint32_t grainSize = 1; // default grain size
    tbb::parallel_for(tbb::blocked_range<int64_t>(
        0, screenHeight, grainSize), TraceImageBody(&data));

    const double dt = stm_sec(stm_since(t0));

    printf("Rendered scene at %ix%i,%ispp in %.3f s\n",
        screenWidth, screenHeight, samplesPerPixel, dt);
    printf("- %.1f K Rays, %.1f K Rays/s\n",
        data.rayCount/1000.0, data.rayCount/1000.0/dt);

    // write resulting image as PNG
    stbi_flip_vertically_on_write(1);
    stbi_write_png("output.png", screenWidth, screenHeight, 4, image.data(), screenWidth*4);

    return 0;
}
