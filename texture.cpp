#include "texture.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

int Texture::LoadFromFile(const char *filename)
{
    data = stbi_load(filename, &w, &h, &comp, 4);
    return (data != nullptr);
}

void Texture::Release()
{
    if (data != nullptr) {
        stbi_image_free(data);
    }

    memset(this, 0, sizeof(Texture));
}

Vector4f Texture::SampleBilinear(Vector2f uv, SamplerWrap wrap) const
{
    if (wrap == SamplerWrap::Repeat) {
        uv.x = fmodf(uv.x, 1.0f);
        uv.y = fmodf(uv.y, 1.0f);
    }

    float dx = uv.x * static_cast<float>(w) - 0.5f;
    float dy = uv.y * static_cast<float>(h) - 0.5f;

    float lx = fmodf(dx, 1.0f);
    float ly = fmodf(dy, 1.0f);
    int xcoord = static_cast<int>(floorf(dx));
    int ycoord = static_cast<int>(floorf(dy));

    Vector4f bl = SampleCoord(xcoord, ycoord);
    Vector4f br = SampleCoord(xcoord+1, ycoord);
    Vector4f tl = SampleCoord(xcoord, ycoord+1);
    Vector4f tr = SampleCoord(xcoord+1, ycoord+1);

    Vector4f bottom = lerp(bl, br, lx);
    Vector4f top = lerp(tl, tr, lx);

    return lerp(bottom, top, ly);
}

int mod(int a, int b)
{
    int r = a % b;
    return r < 0 ? r + b : r;
}

Vector4f Texture::SampleCoord(int xcoord, int ycoord) const {
    Vector4f result = 0.0f;

    xcoord = mod(xcoord, w);
    ycoord = mod(ycoord, h);

    size_t index = (ycoord * w + xcoord) * 4;

    Vector4b tmp = *reinterpret_cast<Vector4b*>(&data[index]);

    result.r = static_cast<float>(tmp.r) / 255.0f;
    result.g = static_cast<float>(tmp.g) / 255.0f;
    result.b = static_cast<float>(tmp.b) / 255.0f;
    result.a = static_cast<float>(tmp.a) / 255.0f;

    return result;
}
