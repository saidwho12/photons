
#include "camera.h"
#include "ray.h"
#include "scene.h"
#include "shape.h"
#include "mesh.h"
#include "bvh.h"

#include <stdlib.h>
#include <iostream>
#include <list>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#if defined(__GNUC__) || defined(__GNUG__) || defined(__clang__)
#define RESTRICT __restrict__
#elif defined(_MSC_VER)
#define RESTRICT __restrict
//#define RESTRICT __declspec(restrict)
#else
#define RESTRICT
#endif

#include <mutex>
#include <thread>

#include <atomic>


enum SplitAxis : uint8_t { SPLIT_X = 0, SPLIT_Y = 1, SPLIT_Z = 2 };
#define KD_TREE_SPLIT_RANDOMLY 0

struct ColourRGBA8 {
    char data[4];
};

struct Photon {
    SphericalDir<Fract16> n,w; // 8 bytes
    Vec3<Fract16> pos; // collision point projected on the world Bounding Volume // 6 bytes
    // HvColour<Fract16> colour;
    Vector3f colour; // 12 bytes
};

Vec3<Fract16> UnprojectFromBounds(const Bounds3f &bounds, const Vector3f &v)
{
    Vector3f u = (v - bounds.v0) / (bounds.v1 - bounds.v0);
    return Vec3<Fract16>{u.x,u.y,u.z};
}

Vector3f ProjectToBounds(const Bounds3f &bounds, float x, float y, float z)
{
    return lerp(bounds.v0, bounds.v1, Vector3f(x,y,z));
}

Vector3f ComputeAlbedo(const Scene *scene, const Material *material, const Hit &hit)
{
    auto albedo = material->albedo;
    const auto &albedoTexture = material->albedoTexture;
    if (albedoTexture.WasSet() && hit.hasUv) {
        const auto *texture = scene->GetTexture(albedoTexture.Get());
        auto col = texture->SampleBilinear(hit.uv, albedoTexture.wrap);
        albedo *= Vector3f(col.r, col.g, col.b);
    }

    return albedo;
}

Vector3f ComputeAmbientOcclusion(const Scene *scene, const Material *material, const Hit &hit)
{
    Vector3f ao (1.0f);

    const auto &aoTexture = material->aoTexture;
    if (aoTexture.WasSet() && hit.hasUv) {
        const auto *texture = scene->GetTexture(aoTexture.Get());
        auto col = texture->SampleBilinear(hit.uv, aoTexture.wrap);
        ao = Vector3f(col.r, col.g, col.b);
    }

    return ao;
}

bool RaytraceScene(const Scene *scene, const Ray &ray, Hit *hit)
{
    // initialize ray tracing  values to defaults
    float nearestT = ray.t2; // start with maximum t value of ray
    Hit nearestHit;
    bool hitAny = false;

    for (const GeometricPrimitive &g : scene->primitives) {
        Hit currHit {};

        if (g.Intersect(ray, &currHit)) {
            if (currHit.t >= ray.t1 && currHit.t < nearestT) {
                nearestT = currHit.t, nearestHit = currHit;
                hitAny = true;
            }
        }
    }

    if (hitAny) {
        if (hit != nullptr)
            *hit = nearestHit;

        return true;
    }

    return false;
}

Vector3f SampleUniformHemisphere()
{
    float u1 = RandFloatInRange(0.0f, 1.0f);
    float u2 = RandFloatInRange(0.0f, 1.0f);
    float r = sqrtf(1.0f - u1 * u1);
    float phi = 2.0f * M_PI * u2;
    return {cos(phi)* r, sin(phi)* r, u1};
}

Vector3f SampleFocusedRay(float theta)
{
    float x0 = sinf(M_PI/2.0f - theta);
    float x1 = 1.0f;
    float u1 = RandUniform() * (x1-x0) + x0;

    float u2 = RandUniform();

    float r = sqrtf(1.0f - u1 * u1);
    float phi = 2.0f * M_PI * u2;
    return {cos(phi)* r, sin(phi)* r, u1};
}

Vector3f CosineSampleHemisphere()
{
    float u1 = RandUniform(), u2 = RandUniform();

    float r = sqrtf(u1);
    float theta = u2 * 2.0f * M_PI;

    float x = r * cosf(theta);
    float y = r * sinf(theta);

    return {x,y,sqrtf(1.0f - u1) };
}

struct KDNode
{
    Bounds3f bounds;

    // data indices are sorted by their position along the split axis, this is a pointer
    // to the area within the higher level indices buffer
    uint64_t start, size;

    SplitAxis axis;
    bool isLeaf;

    // floating point value represents point on split axis children were created from
    float splitDelta;

    KDNode *child1, *child2;
};

struct PhotonMap
{
    Bounds3f worldBound;
    Photon *data;
    size_t numberOfItems;
    uint64_t *sortedOnAxis;

    KDNode *root;
    uint8_t maxItemsPerNode;

    PhotonMap(Bounds3f bound, uint8_t maxNodePhotons)
        : data(nullptr), root(nullptr), sortedOnAxis(nullptr),
    worldBound(bound), maxItemsPerNode(maxNodePhotons),
    numberOfItems(0)
    {}

    void Release()
    {
        if (data != nullptr) {
            free(data);
            numberOfItems = 0;
        }

        if (sortedOnAxis != nullptr) {
            free(sortedOnAxis);
        }
    }

    ~PhotonMap()
    {
        Release();
    }


    bool IsEmpty() const;
    void ComputeKDTree();
    void SavePhoton(const Photon& photon);
    KDNode *ComputeKDTreeChild(Bounds3f nodeBounds, int start, int end);
    std::vector<Photon *> GatherPhotonsWithinRange(const Point3f &p, float r2) const;
    void GatherPhotonsInSphere(std::vector<Photon*> &gathered, KDNode *node,
                      const Point3f& p, float r2) const;
};

// Saves photon in large photon buffer
void PhotonMap::SavePhoton(const Photon& photon)
{
    size_t n = numberOfItems + 1;

    if (data == nullptr) {
        data = (Photon *) malloc(n * sizeof(Photon));
    } else {
        data = (Photon*) realloc(data, n * sizeof (Photon));
    }

    memcpy(&data[numberOfItems], &photon, sizeof photon);
    numberOfItems = n;
}

int ComparePhotonOnXAxis(const Photon *a, const Photon *b);
int ComparePhotonOnYAxis(const Photon *a, const Photon *b);
int ComparePhotonOnZAxis(const Photon *a, const Photon *b);

int ComparePhotonOnXAxis(const Photon *a, const Photon *b)
{
    return a->pos.x <= b->pos.x;
}

int ComparePhotonOnYAxis(const Photon *a, const Photon *b)
{
    return a->pos.y <= b->pos.y;
}

int ComparePhotonOnZAxis(const Photon *a, const Photon *b)
{
    return a->pos.z <= b->pos.z;
}

typedef int (*CMPLE)(const void *, const void *); // less than or equal compare function

void QSort(void *A, size_t itemSize,
           ssize_t start, ssize_t end,
           CMPLE leFn);


void MemorySwap(void *RESTRICT a, void *RESTRICT b, size_t size)
{
    uint8_t *x = static_cast<uint8_t *>(a);
    uint8_t *y = static_cast<uint8_t *>(b);

    for (size_t i = 0; i < size; ++i) {
        uint8_t z = x[i];
        x[i] = y[i];
        y[i] = z;
    }
}

void XORMemorySwap(void *RESTRICT a, void *RESTRICT b, size_t size)
{
    if (a != b) {
        uint8_t *x = static_cast<uint8_t *>(a);
        uint8_t *y = static_cast<uint8_t *>(b);

        for (size_t i = 0; i < size; ++i) {
            x[i] ^= y[i];
            y[i] ^= x[i];
            x[i] ^= y[i];
        }
    }
}

uint64_t QSortPartition(void *A, size_t itemSize,
                        ssize_t start, ssize_t end,
                        CMPLE cmpLe)
{
    uint8_t *basePtr = (uint8_t*)A;

    void *pivot = &basePtr[itemSize*end]; // randomize for better perf
    uint64_t partIdx = start;

    for (uint64_t i = start; i < end; ++i) {
        void *x = &basePtr[itemSize*i]; // A[i]

        if (cmpLe(x, pivot)) {// if A[i] <= pivot
            XORMemorySwap(x, &basePtr[itemSize * partIdx],
                          itemSize);

            ++partIdx;
        }
    }

    XORMemorySwap(&basePtr[itemSize * partIdx],
                  &basePtr[itemSize * end], itemSize);

    return partIdx;
}

void QSort(void *A, size_t itemSize,
           ssize_t start, ssize_t end,
           CMPLE cmpLe)
{
    if (start < end) {
        uint64_t partIdx = QSortPartition(A, itemSize, start, end, cmpLe);
        QSort(A, itemSize, start, partIdx - 1, cmpLe);
        QSort(A, itemSize, partIdx, end, cmpLe);
    }
}

static const CMPLE photonCmpFuncs[3] = {
    reinterpret_cast<CMPLE const>(ComparePhotonOnXAxis),
    reinterpret_cast<CMPLE const>(ComparePhotonOnYAxis),
    reinterpret_cast<CMPLE const>(ComparePhotonOnZAxis)
};

KDNode *PhotonMap::ComputeKDTreeChild(Bounds3f nodeBounds, int start, int end)
{
    KDNode *node = new KDNode;

    node->bounds = nodeBounds;
    size_t size = (end - start) + 1;

    if (size > maxItemsPerNode) {
        // more items than node can fit, split once more
#if KD_TREE_SPLIT_RANDOMLY
        int splitAxis = RandInt() % 3;
#else
        // split on largest dimension
        SplitAxis axis;
        Vector3f dim = nodeBounds.Dim();
        if (dim.x >= dim.y && dim.x >= dim.z)
            axis = SPLIT_X;
        if (dim.y >= dim.x && dim.y >= dim.z)
            axis = SPLIT_Y;
        if (dim.z >= dim.x && dim.z >= dim.y)
            axis = SPLIT_Z;
#endif
        CMPLE cmpLe = photonCmpFuncs[axis];

        // sort photons on split axis, then split on median point
        QSort(data, sizeof(Photon), start, end, cmpLe);
        ssize_t mid = (start + end) / 2;

        // compute children bounding boxes
        Bounds3f bound1, bound2;
        bound1 = bound2 = nodeBounds;

        const Photon *midPoint = &data[mid];
        Vec3<Fract16> f = midPoint->pos;
        Vector3f midPos = ProjectToBounds(worldBound, f.x, f.y, f.z);

        bound1.v1[axis] = midPos[axis];
        bound2.v0[axis] = midPos[axis];

        node->axis = axis;
        node->splitDelta = (midPos[axis] - nodeBounds.v0[axis]);

        node->child1 = ComputeKDTreeChild(bound1, start, mid);
        node->child2 = ComputeKDTreeChild(bound2, mid+1, end);
        node->isLeaf = false;
    } else {
        // this is a leaf node, set flags and item range
        node->child1 = node->child2 = nullptr;
        node->size = size;
        node->start = start;
        node->isLeaf = true;
    }

    return node;
}

// builds KD-Tree nodes from photon array
void PhotonMap::ComputeKDTree()
{
    if (!IsEmpty()) {
        root = ComputeKDTreeChild(worldBound, 0, numberOfItems-1);
    }
}


void
PhotonMap::GatherPhotonsInSphere(std::vector<Photon*> &gathered, KDNode *node,
                                 const Point3f& p, float r2) const
{
    // prune if bounding box distance to point greater than requested radius

    if (!node->isLeaf) { // parent node
        float planeDist = p[node->axis] - (node->bounds.v0[node->axis] + node->splitDelta);

        if (planeDist < 0.0f) { // search child1
            GatherPhotonsInSphere(gathered, node->child1, p, r2);

            // check if second node at least within radius
            if (planeDist * planeDist < r2) {
                GatherPhotonsInSphere(gathered, node->child2, p, r2);
            }
        } else { // search child2
            GatherPhotonsInSphere(gathered, node->child2, p, r2);

            // check if second node at least within radius
            if (planeDist * planeDist < r2) {
                GatherPhotonsInSphere(gathered, node->child1, p, r2);
            }
        }
    } else {
        // leaf node, gather all points with distance less than radius
        uint64_t start = node->start, end = node->start + node->size-1;
        for (uint64_t i = start; i <= end; ++i) {
            auto pos = data[i].pos;
            if (length2(ProjectToBounds(worldBound,pos.x,pos.y,pos.z) - p) <= r2) {
                gathered.push_back(&data[i]);
            }
        }
    }
}

std::vector<Photon *> PhotonMap::GatherPhotonsWithinRange(const Point3f &p, float r2) const
{
    std::vector<Photon *> photons;

    if (!IsEmpty()) {
        if (root->bounds.contains(p)) {
            GatherPhotonsInSphere(photons, root, p, r2);
        }
    }

    return photons;
}

bool PhotonMap::IsEmpty() const {
    return !numberOfItems;
}

struct Frame {
    uint8_t *data;
    uint32_t width, height;
    int comp;
};

Frame *createFrame(int w, int h, int comp)
{
    Frame *frame = new Frame;
    frame->data = new uint8_t [w*h*comp];
    frame->width = w;
    frame->height = h;
    frame->comp = comp;
    return frame;
}

void deleteFrame(Frame *frame)
{
    delete frame->data;
    delete frame;
}

Ray RandomHemisphereRay(Point3f o, Vector3f n, Vector3f (*scatterFunction)() = SampleUniformHemisphere)
{
    Vector3f tangent = normalise(cross(n, Vector3f(n.y,n.z,n.x)));
    Vector3f bitangent = cross(n, tangent);
    Vector3f localDir = scatterFunction();
    Vector3f dir = tangent * localDir.x + bitangent * localDir.y + n * localDir.z;
    return Ray(o + n *RAY_EPSILON,dir);
}


Vector3f SamplePhotonSphere(const Scene *scene, const PhotonMap *photonMap, const Ray &ray, float r2)
{
    Hit hit;
    if (RaytraceScene(scene, ray, &hit)) {
        auto *material = scene->GetMaterial(hit.primitive->GetMaterialID());
        if (!material->isRefractive) {
            Vector3f photonAccum{0.0f};
            auto nearPhotons = photonMap->GatherPhotonsWithinRange(hit.position, r2);

            const float r = sqrtf(r2);
            const float k = 4.0f;

            for (const Photon *p : nearPhotons) {
                auto pos = p->pos;
                auto worldPos = ProjectToBounds(photonMap->worldBound,pos.x,pos.y,pos.z);
                float dp = length(hit.position - worldPos);
                float wpc = 1.0f - dp/(k*r);

                Vector3f Fr = (p->colour);// / M_PI;//* Max(0.0f, dot(p->w, hit.normal));
                photonAccum += Fr * wpc;
            }

            photonAccum /=  M_PI * ((1.0f - (2.0f/(3.0f*k))) * r2);
            return photonAccum;// / (4.0f * hit.t * hit.t * M_PI);//* Max( 0.0f, dot(-ray.d, hit.normal));
        }
    }

    return 0.0f;
}


Vector3f RandomSampleFinalGatherRay(const Scene *scene, const PhotonMap *photonMap, Vector3f o, Vector3f n, float r2)
{
    Ray ray = RandomHemisphereRay(o+n*RAY_EPSILON, n, CosineSampleHemisphere);
    return SamplePhotonSphere(scene, photonMap, ray, r2);
}

// Gets the number of threads the hardware (CPU/Processor) has
static unsigned int GetHwConcurrency()
{
    return std::thread::hardware_concurrency();
}

enum PhotonMapperFlags : uint32_t {
    PHOTON_MAPPER_FLAGS_NONE = 0,
    PHOTON_MAPPER_FLAGS_MULTI_THREADING_BIT = 0x00000001,
    PHOTON_MAPPER_FLAGS_DIRECT_LIGHTING_BIT = 0x00000002,
    PHOTON_MAPPER_FLAGS_INDIRECT_LIGHTING_BIT = 0x00000004,
    PHOTON_MAPPER_FLAGS_CAUSTICS_BIT = 0x00000008,
    PHOTON_MAPPER_FLAGS_ALL_LIGHTING_MASK = PHOTON_MAPPER_FLAGS_DIRECT_LIGHTING_BIT | PHOTON_MAPPER_FLAGS_INDIRECT_LIGHTING_BIT
};

struct PhotonMapperOptions {
    uint32_t flags;
    uint64_t photonCount;
    uint64_t causticPhotonCount;
    unsigned int maxBounce;
    unsigned int sppCount;
    unsigned int fgSamples;
    unsigned int diSamples;
    float radius2;
    float causticsRadius2;
    float gamma;
};

class PhotonMapper {
protected:
    PhotonMapperOptions mOptions;
    PhotonMap *mGlobalMap;
    PhotonMap *mCausticsMap;
    const Scene *mScene;

public:
    PhotonMapper(const Scene *scene, int maxLeafPhotons);
    ~PhotonMapper();

    const Scene *GetScene() const;

    const PhotonMapperOptions *GetOptions() const;
    void SetOptions(PhotonMapperOptions options);

    void Prepass();

    const PhotonMap *GetGlobalMap() const;
    const PhotonMap *GetCausticsMap() const;

    void RenderPixel(Frame *frame, const Camera *camera, short xcoord, short ycoord) const;
};

const Scene *PhotonMapper::GetScene() const {
    return mScene;
}

const PhotonMap *PhotonMapper::GetGlobalMap() const {
    return mGlobalMap;
}

const PhotonMap *PhotonMapper::GetCausticsMap() const {
    return mCausticsMap;
}

void PhotonMapper::SetOptions(PhotonMapperOptions options)
{
    mOptions = options;
}

const PhotonMapperOptions *PhotonMapper::GetOptions() const
{
    return &mOptions;
}

PhotonMapper::PhotonMapper(const Scene *scene, int maxLeafPhotons)
: mScene(scene)
{
    mGlobalMap = new PhotonMap(scene->WorldBound(), maxLeafPhotons);
    mCausticsMap = new PhotonMap(scene->WorldBound(), maxLeafPhotons);
}

PhotonMapper::~PhotonMapper() {
    delete mGlobalMap;
    delete mCausticsMap;
}

// Gathers all caustics generating primitives
static std::vector<GeometricPrimitive> QueryCausticPrimitives(const Scene *scene)
{
    std::vector<GeometricPrimitive> primitives;

    for (const auto &primitive : scene->primitives) {
        const auto material = scene->GetMaterial(primitive.GetMaterialID());
        if (material->isRefractive) {
            primitives.push_back(primitive);
        }
    }

    return primitives;
}

Vector3f ComputeRandomPointInSphere(const Sphere *sphere)
{
    Vector3f center = sphere->objectToWorld.translation;
    float r = sphere->radius;
    float r2 = r*r;

    float x, y, z;

    // loop until random point is inside sphere
    do {
        x = RandFloatInRange(-r, r);
        y = RandFloatInRange(-r, r);
        z = RandFloatInRange(-r, r);
    } while ((x*x + y*y + z*z) >= r2);

    return Vector3f(center.x + x, center.y + y, center.z + z);
}

void PhotonMapper::Prepass() {
    printf("ray tracing and saving bounces ...\n");

    // pick random light, and random ray, bounce and save photons on surface interaction
    for (uint64_t i = 0; i < mOptions.photonCount; ++i) {
        uint32_t lightIndex = RandInt() % mScene->pointLights.size();
        const PointLight &light = mScene->pointLights[lightIndex];
        Ray ray = light.RandomRay();
        Vector3f colour = light.albedo * light.power / static_cast<float>(mOptions.photonCount);

        for (int bounceIndex = 0; bounceIndex < mOptions.maxBounce; ++bounceIndex) {
            Hit hit;

            if (RaytraceScene(mScene, ray, &hit)) {
                // only do diffuse shading and scattering for now
                // choose random scatter direction

                int  matid = hit.primitive->GetMaterialID();
                const Material *material = &mScene->materials.at(matid);

                Ray scatteredRay;

                if (material->isRefractive) {
                    break;
                } else {
                    // Assume always diffuse
                    auto albedo = ComputeAlbedo(mScene, material, hit);

                    // diffuse
                    scatteredRay = RandomHemisphereRay(hit.position, hit.normal, CosineSampleHemisphere);
                    scatteredRay.t1 = 0.0005f;

                    colour *= albedo / M_PI;

                    Vec3<Fract16> pos = UnprojectFromBounds(mGlobalMap->worldBound, hit.position);
                    mGlobalMap->SavePhoton(Photon{hit.normal, scatteredRay.d, pos, colour});
                    ray = scatteredRay;
                }

            } else {
                // Hit void, break out of loop
                break;
            }
        }
    }

    // Select random caustic generating object
    std::vector<GeometricPrimitive> primitives = QueryCausticPrimitives(mScene);

    if (!primitives.empty()) {
        for (uint64_t i = 0; i < mOptions.causticPhotonCount; ++i) {

            // Select random light emitter
            uint32_t lightIndex = RandInt() % mScene->pointLights.size();
            const PointLight &light = mScene->pointLights[lightIndex];
            auto colour = light.albedo * light.power / static_cast<float>(mOptions.causticPhotonCount);

#if 0
            Ray ray = light.RandomRay();
#else

            uint32_t primitiveIndex = RandInt() % primitives.size();

            const auto &primitive = primitives[primitiveIndex];
            const auto *baseShape = primitive.GetShapePointer();

            Vector3f dir;

            switch (baseShape->shapeType) {
                case SHAPE_TYPE_SPHERE: {
                    // Aim rays toward refractive/reflective sphere
                    auto *sphere = reinterpret_cast<const Sphere *>(baseShape);
                    auto randomPoint = ComputeRandomPointInSphere(sphere);
                    dir = normalise(randomPoint - light.position);
                    break;
                }
            }

            Ray ray;
            ray.o = light.position;
            ray.d = dir;
#endif

            bool hitRefractive = false;
            Hit hit;
            for (int bounceIndex = 0; bounceIndex < mOptions.maxBounce; ++bounceIndex) {
                if (RaytraceScene(mScene, ray, &hit)) {
                    // only do diffuse shading and scattering for now
                    // choose random scatter direction

                    const Material *material = mScene->GetMaterial(hit.primitive->GetMaterialID());

                    if (material->isRefractive) {
                        hitRefractive = true;
                        Vector3f R;

                        float eta = hit.frontFace ? (1.0f/material->ior) : material->ior;

                        float cos_theta = fminf(dot(-ray.d, hit.normal), 1.0f);
                        float sin_theta = sqrtf(1.0f - cos_theta*cos_theta);

                        if (eta * sin_theta <= 1.0f) {
                            // can refract
                            R = refract(ray.d, hit.normal, eta);
                            ray.o = hit.position + hit.normal * -RAY_EPSILON;
                        } else {
                            // cannot refract
                            R = reflect(ray.d, hit.normal);
                            ray.o = hit.position + hit.normal * RAY_EPSILON;
                        }

                        float cosT = Max(0.0f, dot(hit.normal,-ray.d));
                        float d2 = powf(eta, 2.0f);
                        colour *= (material->albedo * cosT) / (M_PI * d2);
                        ray.d = normalise(R);
                    } else {
                        // diffuse surface hit, exit loop
                        if (hitRefractive) {
                            Vec3<Fract16> pos = UnprojectFromBounds(mGlobalMap->worldBound, hit.position);
                            mCausticsMap->SavePhoton(Photon{hit.normal, -ray.d, pos, colour});
                        }

                        break;
                    }
                } else {
                    // no intersection, exit
                    break;
                }
            }
        }
    }

    // collect photons
    printf("diffuse photons accumulated: %d\n", mGlobalMap->numberOfItems);
    printf("caustic photons accumulated: %d\n", mCausticsMap->numberOfItems);

    // build kd-tree from photon buffer
    printf("building global photon map ...\n");
    mGlobalMap->ComputeKDTree();
    printf("building caustics photon map ...\n");
    mCausticsMap->ComputeKDTree();
}

Vector3f Tonemapping(Vector3f colour, float exposure)
{
    colour *= exposure;

    float A = 0.15f;
    float B = 0.50f;
    float C = 0.10f;
    float D = 0.20f;
    float E = 0.02f;
    float F = 0.30f;
    float W = 11.2f;
    colour = ((colour * (A * colour + C * B) + D * E) / (colour * (A * colour + B) + D * F)) - E / F;
    float white = ((W * (A * W + C * B) + D * E) / (W * (A * W + B) + D * F)) - E / F;
    colour /= white;

    return colour;
}

Vector3f LinearToSRGB(Vector3f colour, float gamma)
{
    Vector3f sRGBLo = colour * 12.92f;
    const float powExp = 1.0f/gamma; // power ramp for gamma correction
    Vector3f sRGBHi = (Vector3f(powf(fabsf(colour.x), powExp),
                                powf(fabsf(colour.y), powExp),
                                powf(fabsf(colour.z), powExp)) * 1.055f) - 0.055f;
    Vector3f sRGB;
    sRGB.x = (colour.x <= 0.0031308f) ? sRGBLo.x : sRGBHi.x;
    sRGB.y = (colour.y <= 0.0031308f) ? sRGBLo.y : sRGBHi.y;
    sRGB.z = (colour.z <= 0.0031308f) ? sRGBLo.z : sRGBHi.z;

    return sRGB;
}

void PhotonMapper::RenderPixel(Frame *frame, const Camera *camera,
               short xcoord, short ycoord) const
{
    uintptr_t index = (uintptr_t)ycoord * frame->width + (uintptr_t)xcoord;
    Vector3f finalColour;

    for (int sampleIndex = 0; sampleIndex < mOptions.sppCount; ++sampleIndex) {
        Vector2f msaaDelta = {RandUniform(), RandUniform()};

        // random offset into the pixel for MSAA sample
        Vector2f uv = {((float)xcoord + msaaDelta.x) /(float)frame->width,
                       ((float)ycoord + msaaDelta.y) /(float)frame->height};

        Ray ray = camera->GetRayAtPosition(uv);

        if (mOptions.flags & PHOTON_MAPPER_FLAGS_DIRECT_LIGHTING_BIT) {
            // Compute direct illumination
            Vector3f direct;
            Ray currRay = ray;

            Vector3f falloff = Vector3f(1.0f);

            for (int bounceIdx = 0; bounceIdx < mOptions.maxBounce; ++bounceIdx) {

                Hit hit;
                if (RaytraceScene(mScene, currRay, &hit)) {
                    const Material *material = mScene->GetMaterial(hit.primitive->GetMaterialID());

                    if (material->isRefractive) {
                        Vector3f R;

                        // n1/n2 if front face, otherwise n2/n1
                        float eta = hit.frontFace ? (1.0f/material->ior) : material->ior;

                        float cos_theta = fminf(dot(-currRay.d, hit.normal), 1.0f);
                        float sin_theta = sqrtf(1.0f - cos_theta*cos_theta);

                        if (eta * sin_theta <= 1.0f) {
                            // can refract
                            R = refract(currRay.d, hit.normal, eta);
                            currRay.o = hit.position + hit.normal * -RAY_EPSILON;
                        } else {
                            // cannot refract
                            R = reflect(currRay.d, hit.normal);
                            currRay.o = hit.position + hit.normal * RAY_EPSILON;
                        }

                        currRay.d = normalise(R);
                        falloff *= material->albedo;
                    } else {
                        // Diffuse material
                        Vector3f lightSum;

                        for (const PointLight& l: mScene->pointLights) {
                            Vector3f surfaceToLight = l.position-hit.position;
                            float d = length(surfaceToLight);
                            Vector3f L = normalise(surfaceToLight);

                            Hit shadowHit;
                            Ray shadowRay(hit.position, L, RAY_EPSILON, d);

                            if (!RaytraceScene(mScene, shadowRay, &shadowHit)) {
                                // If not in shadow, compute lighting from light source
                                // with correct falloff
                                // using inverse square law.
                                float NoL = Max(0.0f, dot(hit.normal, L));
                                float ramp = l.power / (4.0f * M_PI * powf(d,2.0f));

                                Vector3f diffuse = l.albedo * NoL * ramp;
                                lightSum += diffuse;
                            }

                        }

                        auto ao = ComputeAmbientOcclusion(mScene, material, hit);
                        auto albedo = ComputeAlbedo(mScene, material, hit);
                        direct = lightSum * albedo * ao * falloff;
                        break;
                    }
                } else {
                    direct = mScene->SampleEnv(currRay.d);
                }
            }

            finalColour += direct;
        }

        if (mOptions.flags & PHOTON_MAPPER_FLAGS_INDIRECT_LIGHTING_BIT) {
            Vector3f indirect;
            Hit hit;

            if (RaytraceScene(mScene, ray, &hit)) {
                // compute indirect illumination
                const auto *material = mScene->GetMaterial(hit.primitive->GetMaterialID());

                if (!material->isRefractive) {
                    // if material is not refractive, integrate radiance coming from hemisphere
                    for (int i = 0; i < mOptions.fgSamples; ++i) {
                        indirect += RandomSampleFinalGatherRay(mScene, mGlobalMap, hit.position, hit.normal,
                                                               mOptions.radius2);
                    }

                    auto albedo = ComputeAlbedo(mScene, material, hit);
                    indirect *= albedo / static_cast<float>(mOptions.fgSamples);
                }
            }

            finalColour += indirect;
        }

        if (mOptions.flags & PHOTON_MAPPER_FLAGS_CAUSTICS_BIT) {
            Vector3f caustics;
            caustics = SamplePhotonSphere(mScene, mCausticsMap, ray, mOptions.causticsRadius2);
            finalColour += caustics;
        }
    }

    finalColour /= static_cast<float>(mOptions.sppCount);

    finalColour = Tonemapping(finalColour, 3.0f);
    finalColour = LinearToSRGB(finalColour, mOptions.gamma);

    frame->data[index*4+0] = (uint8_t)floorf(clamp(finalColour.x, 0.0f, 1.0f) * 255.0f);
    frame->data[index*4+1] = (uint8_t)floorf(clamp(finalColour.y, 0.0f, 1.0f) * 255.0f);
    frame->data[index*4+2] = (uint8_t)floorf(clamp(finalColour.z, 0.0f, 1.0f) * 255.0f);
    frame->data[index*4+3] = 255;
}

enum JobStatus : uint8_t {
    JOB_STATUS_OPEN = 0,
    JOB_STATUS_IN_PROGRESS = 0x01,
    JOB_STATUS_FINISHED = 0x02
};

struct Bounds2h {
    int16_t x0;
    int16_t y0;
    int16_t x1;
    int16_t y1;

    Bounds2h() {}
};

struct TileJob {
    Bounds2h bounds; JobStatus status;
    TileJob() : status(JOB_STATUS_OPEN) { }
};

class AsyncRenderer {
    PhotonMapper *mapper;
    uint16_t tileWidth, tileHeight;

    std::vector<TileJob> jobs;
    std::atomic<int32_t> firstOpenJob;
    std::mutex grantJobMutex;
    std::atomic<int32_t> finishedJobs;

    TileJob *GrantJob();
    void PrepareJobs(const Frame *frame);
    void ExecuteJobs(const Scene *scene,  unsigned int workerId, Frame *frame, const Camera *camera);

 public:
    AsyncRenderer(PhotonMapper *m, short tw, short th)
    : mapper(m), tileWidth(tw), tileHeight(th), firstOpenJob(0), finishedJobs(0)
    {}

    virtual ~AsyncRenderer() = default;

    void Render(Frame *frame, const Camera *camera, unsigned int threadCount = GetHwConcurrency());
};

void AsyncRenderer::PrepareJobs(const Frame *frame)
{
    // Split frame into many tiles
    for (int16_t ycoord = 0; ycoord < frame->height; ycoord += tileHeight) {
        for (int16_t xcoord = 0; xcoord < frame->width; xcoord += tileWidth) {
            TileJob job;
            job.bounds.x0 = xcoord;
            job.bounds.x1 = Min<int16_t>(frame->width, xcoord + tileWidth);
            job.bounds.y0 = ycoord;
            job.bounds.y1 = Min<int16_t>(frame->height, ycoord + tileHeight);

            jobs.push_back(job);
        }
    }
}

void AsyncRenderer::ExecuteJobs(const Scene *scene, unsigned int workerId,
                                Frame *frame, const Camera *camera)
{
    TileJob *job;

    while ((job = GrantJob()) != nullptr) {
        // Render the current given tile
        for (int16_t ycoord = job->bounds.y0; ycoord < job->bounds.y1; ++ycoord) {
            for (int16_t xcoord = job->bounds.x0; xcoord < job->bounds.x1; ++xcoord) {
                mapper->RenderPixel(frame, camera, xcoord, ycoord);
            }
        }

        ++finishedJobs;
        printf("Worker %d finished: %d/%d\n", workerId, finishedJobs.load(), jobs.size());
        job->status = JOB_STATUS_FINISHED;
    }
}

void AsyncRenderer::Render(Frame *frame, const Camera *camera, unsigned int threadCount)
{
    std::vector<std::thread> threads(threadCount);
    PrepareJobs(frame);
    auto *scene = mapper->GetScene();
    mapper->Prepass();

    printf("Rendering with %d threads.\n", threadCount);

    for (int threadIdx = 0; threadIdx < threadCount; ++threadIdx) {
        threads.emplace_back(&AsyncRenderer::ExecuteJobs, this, scene, threadIdx, frame, camera);
    }

    // wait for all jobs to be finished
    while (finishedJobs < jobs.size()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    for (auto &thread: threads) {
        if (thread.joinable()) thread.join();
    }

    threads.clear();

    printf("All jobs finished.\n");
}

TileJob *AsyncRenderer::GrantJob() {
    std::lock_guard<std::mutex> guard(grantJobMutex);

    if (firstOpenJob == -1) {
        return nullptr;
    }

    TileJob *job = &jobs[firstOpenJob];
    job->status = JOB_STATUS_IN_PROGRESS;

    // Find next open index
    while (jobs[firstOpenJob].status
    & (JOB_STATUS_IN_PROGRESS | JOB_STATUS_FINISHED)) {
        if (firstOpenJob >= jobs.size()) {
            firstOpenJob = -1; break;
        }

        ++firstOpenJob;
    }

    return job;
}

void render(Camera *camera, Scene *scene, uint32_t width, uint32_t height)
{
    Frame *frame = createFrame(width, height, 4);
    PhotonMapper photonMapper(scene, 8);

    PhotonMapperOptions options = {
            PHOTON_MAPPER_FLAGS_DIRECT_LIGHTING_BIT|PHOTON_MAPPER_FLAGS_INDIRECT_LIGHTING_BIT|PHOTON_MAPPER_FLAGS_CAUSTICS_BIT,
            200000, // photon count
            0, // caustic photon count
            6, // max number of bounces
            4, // samples per pixel
            8, // final gather samples
            8, // direct illumination samples (ni)
            0.05f, // radius squared
            0.0025f, // radius squared for caustics
            2.4f // gamma
    };

    photonMapper.SetOptions(options);

    AsyncRenderer renderer(&photonMapper, 64, 64);
    renderer.Render(frame, camera, 8);

    {
        // Write the frame out to output folder with unique timestamp
        char filename[400];
        time_t now = time ( &now );
        struct tm timeinfo;
        localtime_s(&timeinfo, &now);

        sprintf(filename, "../output/frame-%d-%d-%d.%d-%d-%d.png",
                timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

        stbi_flip_vertically_on_write(1);
        if (!stbi_write_png(filename, frame->width, frame->height, frame->comp, frame->data, 0)) {
            fprintf(stderr, "%s\n", "Failed to write image!");
        }
    }

    deleteFrame(frame);
}

int main(int argc, char *argv[])
{
    BvhTree dragon_bvh;
    ply_mesh dragon;
    const char *mesh_path = "../data/stanford_dragon_vrip_decimated.ply";
    load_ply_mesh(dragon, mesh_path);
    printf("Building BVH tree for mesh \"%s\"...\n", mesh_path);
    dragon_bvh.BuildBvh(dragon);
    printf("Built BVH tree for mesh \"%s\"\n", mesh_path);

    uint32_t width = 2000, height = 2000;
    float aspect = (float)width/(float)height;

    Camera camera = Camera(Point3f(0.64f, 1.41f, 2.32f), Point3f(0.0f, 0.0f, 0.0f), aspect, 90.0f);

    // setup scene
    Scene scene;
    scene.skyColour = Vector3f(0.5f, 0.5f, 0.92f);
    Material sandMaterial(
            Vector3f(1.0f, 1.0f, 1.0f),
            0.0f,
            1.0f,
            1.0f,
            false);

    scene.AddPrimitive(new MeshInstance(&dragon_bvh,Transform(Vector3f{0.0f,0.0f,0.0f})),
        Material(Vector3f(1.0f, 1.0f, 1.0f), 0.0f, 1.0f, 1.0f));

    // {
    //     Texture albedo;
    //     if (albedo.LoadFromFile("../data/TexturesCom_SoilSand0092_1_seamless_S.jpg")) {
    //         int textureId = scene.AddTexture(albedo);
    //         sandMaterial.albedoTexture.Set(textureId);
    //         sandMaterial.albedoTexture.filter = SamplerFilter::Bilinear;
    //         sandMaterial.albedoTexture.wrap = SamplerWrap::Repeat;
    //     } else {
    //         printf("Failed to load texture!\n");
    //     }
    // }

    // scene.AddPrimitive(Sphere(Vector3f(0.0f, -1.0f, 2.75f), 1.0f),
    //                    sandMaterial);

    // scene.AddPrimitive(Sphere(Vector3f(1.0f, -1.0f, 1.0f), 1.0f),
    //                    Material(Vector3f(1.0f, 0.0f, 0.0f), 0.0f, 1.0f, 1.0f));

    // scene.AddPrimitive(Sphere(Vector3f(-1.0f, -1.0f, 1.0f), 1.0f),
    //                    Material(Vector3f(1.0f, 1.0f, 1.0f), 0.0f, 1.0f, 1.0f));

    scene.AddPrimitive(new Sphere(Vector3f(0.35f, 0.82f, 0.82f), 0.44f),
                       Material(Vector3f(1.0f, 1.0f, 1.0f), 0.5f, 1.0f, 1.5f, true));


    scene.AddPrimitive(new Disk(Vector3f(0.0f, 0.0f, 0.0f),Vector3f(0.0f, 1.0f, 0.0f), 10.0f),
                       Material(Vector3f(1.0f, 1.0f, 1.0f), 0.0f, 1.0f, 1.0f));
    

    // scene.AddPrimitive(Disk(Vector3f(0.0f, 0.0f, 0.0f),Vector3f(0.0f, -1.0f, 0.0f), 10.0f),
    //                    Material(Vector3f(1.0f, 1.0f, 1.0f), 0.0f, 1.0f, 1.0f));

    scene.AddPointLight(PointLight(Vector3f(0.73f, 3.5f, 3.8f),
                                   Vector3f(1.0f), 150.0f ));



    render(&camera, &scene, width, height);

    return 0;
}
