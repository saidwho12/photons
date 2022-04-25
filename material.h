#pragma once

#include <array>
#include "texture.h"

enum BxDFType : int32_t {
    BXDF_TYPE_DIFFUSE = 0x00000001,
    BXDF_TYPE_SPECULAR = 0x00000002,
};

// the following BxDF class holds an interface to a
//
class BxDF {
public:
    virtual ~BxDF() { }
    BxDF(BxDFType type) : type(type) { }

    virtual float Pdf(const Vector3f &wi, const Vector3f &wo) const;
    virtual float f(const Vector3f &wi, const Vector3f &wo) const;

    BxDFType type;
};

static constexpr int MAX_BXDF = 16;

class BSDF {

    std::array<BxDF*, MAX_BXDF> bxdfs;
};

#define AIR_IOR 1.000293
#define WATER_IOR 1.333
#define GLASS_IOR 1.500

enum MaterialType : int32_t {
    MATERIAL_TYPE_EMISSIVE,
    MATERIAL_TYPE_METAL,
    MATERIAL_TYPE_DIELECTRIC,
    MATERIAL_TYPE_MIRROR,
};

static Vector3f SchlickFresnel(const float &NoX, const Vector3f &F0)
{
    return F0 + (1.0f - F0) * powf(1.0f - NoX, 5.0f);
}

struct Material {
    Vector3f albedo;
    Sampler2D albedoTexture;
    Sampler2D aoTexture;

    float kS; // between 0 and 1
    float ior; // >= 0
    float roughness; // between 0 and 1
    bool isRefractive;

    Material(Vector3f colour, float specularity, float roughness, float ior = 1.0f, bool isRefractive = false)
    : albedo(colour), kS(specularity), roughness(roughness), ior(ior), isRefractive(isRefractive)
    {}

    inline auto ComputeF0() const -> Vector3f {
        return powf(fabsf((1.0f - ior)/(1.0f + ior)), 2.0f);
    }
};
