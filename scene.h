
#pragma once

#include "shape.h"
#include "lighting.h"
#include "texture.h"

#include <cstdint>
#include <vector>
#include <unordered_map>
#include <map>

struct Scene {
    Vector3f skyColour;
    std::unordered_map<int, Texture> textures;
    std::unordered_map<int, Material> materials;

    std::vector<PointLight> pointLights;
    std::vector<GeometricPrimitive> primitives;

    static int textureIdCounter;
    static int materialIdCounter;
    static int objectIdCounter;
    static int NextMaterialID() { return materialIdCounter++; }
    static int NextObjectID() { return objectIdCounter++; }
    static int NextTextureID() { return textureIdCounter++; }

    Scene();
    ~Scene() {}

    int AddTexture(Texture texture);
    Texture *GetTexture(int textureId) const;

    const Material *GetMaterial(int id) const;
    int AddMaterial(Material m);

    void AddPrimitive(Shape *shape, const Material &m);
    void AddPointLight(const PointLight &light);


    Vector3f SampleEnv(const Vector3f &v) const;
    Bounds3f WorldBound() const;
};
