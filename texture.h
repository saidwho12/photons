#pragma once

#include "math.h"

enum SamplerWrap : uint8_t {
    Repeat, Clamp, ClampToEdge
};

enum SamplerFilter : uint8_t {
    Nearest, Bilinear, Bicubic
};

struct Sampler2D {
    Sampler2D() : wasSet(false) {}
    ~Sampler2D() {}

    inline const uint32_t &Get() const { return id; }
    inline void Set(uint32_t textureId) { id = textureId; wasSet = true; }
    inline bool WasSet() const { return wasSet; }

    SamplerWrap wrap;
    SamplerFilter filter;
private:
    bool wasSet;
    uint32_t id;
};

struct Texture {
    uint8_t *data;
    int w, h, comp;

    Texture(): data(nullptr) { comp = w = h = 0; }
    Vector4f SampleCoord(int xcoord, int ycoord) const;

    Vector4f SampleBilinear(Vector2f uv, SamplerWrap wrap) const;
    int LoadFromFile(const char *filename);
    void Release();
};

