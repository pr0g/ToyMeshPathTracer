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
#include "tbb/blocked_range2d.h"
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
static const rtm::vector4f kLightDir =
    rtm::vector_normalize3(
        rtm::vector_set(-0.7f, 1.0f, 0.5f, 0.0f),
        rtm::vector_set(0.0f));
static const rtm::vector4f kLightColor =
    rtm::vector_set(0.7f,0.6f,0.5f, 1.0f);


// when a ray "r" has just hit a surface at point "hit", decide what to do about it:
// in our very simple case, we assume the surface is perfectly diffuse, so we'll return:
// - surface albedo ("color") in "attenuation"
// - new random ray for the next light bounce in "scattered"
// - illumination from the directional light in "outLightE"
static bool Scatter(
    const Ray& r, const Hit& hit, rtm::vector4f& attenuation,
    Ray& scattered, rtm::vector4f& outLightE, uint32_t& rngState, int& inoutRayCount)
{
    outLightE = rtm::vector_set(0.0f);

    // model a perfectly diffuse material:
    
    // random point on unit sphere that is tangent to the hit point
    rtm::vector4f target =
        rtm::vector_add(
            hit.pos,
            rtm::vector_add(
                hit.normal,
                RandomUnitVector(rngState)));
    
    scattered =
        Ray(
            hit.pos,
            rtm::vector_normalize3(
                rtm::vector_sub(
                    target,
                    hit.pos),
                rtm::vector_set(0.0f)));
    
    // make color slightly based on surface normals
    rtm::vector4f albedo = hit.normal * 0.0f + rtm::vector_set(0.7f,0.7f,0.7f, 0.0f); //BUG?
    attenuation = albedo;
    
    // explicit directional light by shooting a shadow ray
    ++inoutRayCount;
    Hit lightHit;
    int id = HitScene(Ray(hit.pos, kLightDir), kMinT, kMaxT, lightHit);
    if (id == -1)
    {
        // ray towards the light did not hit anything in the scene, so
        // that means we are not in shadow: compute illumination from it
        rtm::vector4f rdir = r.dir;
        AssertUnit(rdir);
        rtm::vector4f nl = rtm::vector_dot(hit.normal, rdir) < 0.0f ? hit.normal : -hit.normal; // use select
        
        outLightE = rtm::vector_add(
            outLightE,
            rtm::vector_mul(
                albedo,
                rtm::vector_mul(
                    kLightColor,
                    fmaxf(
                        0.0f,
                        rtm::vector_dot3(
                            kLightDir,
                            nl)))));
        
        //outLightE += albedo * kLightColor * (fmax(0.0f, dot(kLightDir, nl)));
    }

    return true;
}


// trace a ray into the scene, and return the final color for it
static rtm::vector4f Trace(const Ray& r, int depth, uint32_t& rngState, int& inoutRayCount)
{
    ++inoutRayCount;
    Hit hit;
    int id = HitScene(r, kMinT, kMaxT, hit);
    if (id != -1)
    {
        // ray hits something in the scene
        Ray scattered;
        rtm::vector4f attenuation;
        rtm::vector4f lightE;
        if (depth < kMaxDepth && Scatter(r, hit, attenuation, scattered, lightE, rngState, inoutRayCount))
        {
            // we got a new ray bounced from the surface; recursively trace it
            return rtm::vector_add(lightE, rtm::vector_mul(attenuation, Trace(scattered, depth+1, rngState, inoutRayCount)));
        }
        else
        {
            // reached recursion limit, or surface fully absorbed the ray: return black
            return rtm::vector_set(0.0f);
        }
    }
    else
    {
        // ray does not hit anything: return illumination from the sky (just a simple gradient really)
        float t = 0.5f * (rtm::vector_get_y(r.dir) + 1.0f);
        return
            rtm::vector_mul(
                rtm::vector_add(
                    rtm::vector_mul(rtm::vector_set(1.0f), 1.0f - t),
                    rtm::vector_mul(rtm::vector_set(0.5f, 0.7f, 1.0f), t)),
                0.5f);
    }
}


// load scene from an .OBJ file
static bool LoadScene(const char* dataFile, rtm::vector4f& outBoundsMin, rtm::vector4f& outBoundsMax)
{
    ObjFile objFile;
    if (!objParseFile(objFile, dataFile))
    {
        printf("ERROR: failed to load .obj file\n");
        return false;
    }
    outBoundsMin = rtm::vector_set(+1.0e6f, +1.0e6f, +1.0e6f, 0.0f);
    outBoundsMax = rtm::vector_set(-1.0e6f, -1.0e6f, -1.0e6f, 0.0f);

    int objTriCount = int(objFile.f_size / 9);
    Triangle* tris = new Triangle[objTriCount + 2]; // will add two triangles for the "floor"
    for (int i = 0; i < objTriCount; ++i)
    {
        int idx0 = objFile.f[i * 9 + 0] * 3;
        int idx1 = objFile.f[i * 9 + 3] * 3;
        int idx2 = objFile.f[i * 9 + 6] * 3;
        rtm::vector4f v0 = rtm::vector_set(objFile.v[idx0 + 0], objFile.v[idx0 + 1], objFile.v[idx0 + 2]);
        rtm::vector4f v1 = rtm::vector_set(objFile.v[idx1 + 0], objFile.v[idx1 + 1], objFile.v[idx1 + 2]);
        rtm::vector4f v2 = rtm::vector_set(objFile.v[idx2 + 0], objFile.v[idx2 + 1], objFile.v[idx2 + 2]);
        tris[i].v0 = v0;
        tris[i].v1 = v1;
        tris[i].v2 = v2;
        
        outBoundsMin = rtm::vector_min(outBoundsMin, v0); outBoundsMax = rtm::vector_max(outBoundsMax, v0);
        outBoundsMin = rtm::vector_min(outBoundsMin, v1); outBoundsMax = rtm::vector_max(outBoundsMax, v1);
        outBoundsMin = rtm::vector_min(outBoundsMin, v2); outBoundsMax = rtm::vector_max(outBoundsMax, v2);
    }

    // add two triangles that are right "under the scene" and covering larger area than the scene
    // itself, to serve as a "floor"
    rtm::vector4f size = rtm::vector_sub(outBoundsMax, outBoundsMin);
    rtm::vector4f extra = rtm::vector_mul(size, 0.7f);
    tris[objTriCount].v0 =
        rtm::vector_set(
            rtm::vector_get_x(outBoundsMin) - rtm::vector_get_x(extra),
            rtm::vector_get_y(outBoundsMin),
            rtm::vector_get_z(outBoundsMin) - rtm::vector_get_z(extra));
    tris[objTriCount].v1 =
        rtm::vector_set(
            rtm::vector_get_x(outBoundsMin) - rtm::vector_get_x(extra),
            rtm::vector_get_y(outBoundsMin),
            rtm::vector_get_z(outBoundsMax) + rtm::vector_get_z(extra));
    tris[objTriCount].v2 =
        rtm::vector_set(
            rtm::vector_get_x(outBoundsMax) + rtm::vector_get_x(extra),
            rtm::vector_get_y(outBoundsMin),
            rtm::vector_get_z(outBoundsMin) - rtm::vector_get_z(extra));
    tris[objTriCount+1].v0 =
        rtm::vector_set(
            rtm::vector_get_x(outBoundsMin) - rtm::vector_get_x(extra),
            rtm::vector_get_y(outBoundsMin),
            rtm::vector_get_z(outBoundsMax) + rtm::vector_get_z(extra));
    tris[objTriCount+1].v1 =
        rtm::vector_set(
            rtm::vector_get_x(outBoundsMax) + rtm::vector_get_x(extra),
            rtm::vector_get_y(outBoundsMin),
            rtm::vector_get_z(outBoundsMax) + rtm::vector_get_z(extra));
    tris[objTriCount+1].v2 =
        rtm::vector_set(
            rtm::vector_get_x(outBoundsMax) + rtm::vector_get_x(extra),
            rtm::vector_get_y(outBoundsMin),
            rtm::vector_get_z(outBoundsMin) - rtm::vector_get_z(extra));

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
	{
	}
	
	void operator()(const tbb::blocked_range2d<uint32_t>& range) const {
		TraceData& data = *m_traceData;
		float invWidth = m_invWidth;
		float invHeight = m_invHeight;
		uint8_t* image = data.image;
		
		int rayCount = 0;
		for (uint32_t y = range.rows().begin(); y != range.rows().end(); ++y)
		{
			uint32_t rngState = y * 9781 + 1;
			for (uint32_t x = range.cols().begin(); x != range.cols().end(); ++x)
			{
				rtm::vector4f col = rtm::vector_set(0.0f);
				// we'll trace N slightly jittered rays for each pixel, to get anti-aliasing, loop over them here
				for (int s = 0; s < data.samplesPerPixel; s++)
				{
					// get a ray from camera, and trace it
					float u = float(x + RandomFloat01(rngState)) * invWidth;
					float v = float(y + RandomFloat01(rngState)) * invHeight;
					Ray r = data.camera->GetRay(u, v, rngState);
					col = rtm::vector_add(col, Trace(r, 0, rngState, rayCount));
				}
				
				col *= 1.0f / float(data.samplesPerPixel);
				
				// simplistic "gamma correction" by just taking a square root of the final color
				col = rtm::vector_set(
                    sqrtf(rtm::vector_get_x(col)),
				    sqrtf(rtm::vector_get_y(col)),
                    sqrtf(rtm::vector_get_z(col)));
				
				// our image is bytes in 0-255 range, turn our floats into them here and write into the image
				const uint32_t lookup = (y * data.screenWidth + x) * 4;
				image[lookup + 0] = uint8_t(saturate(rtm::vector_get_x(col)) * 255.0f);
				image[lookup + 1] = uint8_t(saturate(rtm::vector_get_y(col)) * 255.0f);
				image[lookup + 2] = uint8_t(saturate(rtm::vector_get_z(col)) * 255.0f);
				image[lookup + 3] = 255;
			}
		}

		data.rayCount += rayCount;
	}
	
private:
	TraceData* m_traceData = nullptr;
	float m_invWidth = 0.0f;
	float m_invHeight = 0.0f;
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
    rtm::vector4f sceneMin, sceneMax;
    if (!LoadScene("/Users/tomhultonharrop/Documents/Projects/ray-tracing-interview/data/suzanne.obj"/*argv[4]*/, sceneMin, sceneMax))
        return 1;

    // place a camera: put it a bit outside scene bounds, looking at the center of it
    rtm::vector4f sceneSize = rtm::vector_sub(sceneMax, sceneMin);
    rtm::vector4f sceneCenter = rtm::vector_mul(rtm::vector_add(sceneMin,sceneMax), 0.5f);
    rtm::vector4f lookfrom = rtm::vector_add(sceneCenter, rtm::vector_mul(sceneSize, rtm::vector_set(0.3f,0.6f,1.2f)));
    if (strstr(argv[4], "sponza.obj") != nullptr) // sponza looks bad when viewed from outside; hardcode camera position
        lookfrom = rtm::vector_set(-5.96f, 4.08f, -1.22f);
    rtm::vector4f lookat = rtm::vector_add(sceneCenter, rtm::vector_mul(sceneSize, rtm::vector_set(0.0f,-0.1f,0.0f)));
    float distToFocus = rtm::vector_length3(rtm::vector_sub(lookfrom, lookat));
    float aperture = 0.03f;
    auto camera = Camera(lookfrom, lookat, rtm::vector_set(0.0f, 1.0f, 0.0f), 60.0f, float(screenWidth) / float(screenHeight), aperture, distToFocus);

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
	
	const uint32_t grainSize = 10000; // disable threading
	tbb::parallel_for(tbb::blocked_range2d<uint32_t>(
		0, screenHeight, grainSize, 0, screenWidth, grainSize), TraceImageBody(&data));

    double dt = stm_sec(stm_since(t0));
    printf("Rendered scene at %ix%i,%ispp in %.3f s\n", screenWidth, screenHeight, samplesPerPixel, dt);
    printf("- %.1f K Rays, %.1f K Rays/s\n", data.rayCount/1000.0, data.rayCount/1000.0/dt);

    // write resulting image as PNG
    stbi_flip_vertically_on_write(1);
    stbi_write_png("output.png", screenWidth, screenHeight, 4, image.data(), screenWidth*4);

    return 0;
}
