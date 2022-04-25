
#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <cfloat>
#include <cstdint>

struct Vector3f {
    float x, y, z;

    Vector3f() : x(0.0f), y(0.0f), z(0.0f) {};
    Vector3f(float v) : x(v), y(v), z(v) {}
    Vector3f(float px, float py, float pz)
    {
        x = px;
        y = py;
        z = pz;
    }

    float operator [](int i) const { return reinterpret_cast<const float*>(this)[i]; }
    float& operator [](int i) { return reinterpret_cast<float*>(this)[i]; }

    Vector3f operator +(const float &other) const {
        return Vector3f(x + other, y + other, z + other);
    }

    Vector3f operator -(const float &other) const {
        return Vector3f(x - other, y - other, z - other);
    }

    Vector3f operator *(const float &other) const {
        return Vector3f(x * other, y * other, z * other);
    }

    Vector3f operator /(const float &other) const {
        return Vector3f(x / other, y / other, z / other);
    }

    Vector3f operator /(const Vector3f &other) const {
        return Vector3f(x / other.x, y / other.y, z / other.z);
    }

    Vector3f operator +=(const Vector3f &other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3f operator -=(const Vector3f &other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3f operator *=(const Vector3f &other) {
        x *= other.x;
        y *= other.y;
        z *= other.z;
        return *this;
    }

    Vector3f operator /=(const Vector3f &other) {
        x /= other.x;
        y /= other.y;
        z /= other.z;
        return *this;
    }

    void normalise()
    {
        float len = (float)sqrt((double)(x*x + y*y + z*z));
        x = x / len;
        y = y / len;
        z = z / len;
    }

    Vector3f normalised() const {
        float len = (float)sqrt((double)(x*x + y*y + z*z));
        return {x / len, y / len, z / len};
    }

    float dot(Vector3f &other) {
        return x*other.x + y*other.y + z*other.z;
    }

    void negate() {
        x = -x;
        y = -y;
        z = -z;
    }

//	void cross(Vector &other, Vector &result)
//	{
//	  result.x = y*other.z - z*other.y;
//	  result.y = z*other.x - x*other.z;
//	  result.z = x*other.y - y*other.x;
//	}

    Vector3f cross(Vector3f const& other) const
    {
        return { y*other.z - z*other.y,
                 z*other.x - x*other.z,
                 x*other.y - y*other.x
        };
    }

    void scale(float s ){
        x *= s;
        y *= s;
        z *= s;
    }

    float mag() {
        return sqrt(x * x + y * y + z * z);
    }

    Vector3f operator-() const
    {
        return {-x,-y,-z};
    }
};

static Vector3f operator+ (const Vector3f& a,const Vector3f& b)
{
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

static Vector3f operator- (const Vector3f& a,const Vector3f& b)
{
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

static Vector3f operator* (const Vector3f& a,const Vector3f& b)
{
    return {a.x * b.x, a.y * b.y, a.z * b.z};
}

using Point3f = Vector3f;

struct Vector2f {
    float x, y;

    Vector2f() = default;
    Vector2f(float px, float py) : x(px), y(py) {}
    ~Vector2f() {}


    Vector2f operator +(const float &other) const {
        return Vector2f(x + other, y + other);
    }

    Vector2f operator -(const float &other) const {
        return Vector2f(x - other, y - other);
    }

    Vector2f operator *(const float &other) const {
        return Vector2f(x * other, y * other);
    }

    Vector2f operator /(const float &other) const {
        return Vector2f(x / other, y / other);
    }

    Vector2f operator +(const Vector2f &other) const {
        return Vector2f(x + other.x, y + other.y);
    }

    Vector2f operator -(const Vector2f &other) const {
        return Vector2f(x - other.x, y - other.y);
    }

    Vector2f operator *(const Vector2f &other) const {
        return Vector2f(x * other.x, y * other.y);
    }

    Vector2f operator /(const Vector2f &other) const {
        return Vector2f(x / other.x, y / other.y);
    }
};

static float length(const Vector3f &v) {
    return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

static float length2(const Vector3f &v) {
    return v.x*v.x + v.y*v.y + v.z*v.z;
}

static Vector3f normalise(const Vector3f &v) {
    float len = length(v);
    return v / len;
}

static Vector3f cross(const Vector3f& a, const Vector3f& b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

static float dot(const Vector3f& a, const Vector3f& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

static float Radians(float deg) {
    return (deg * M_PI)/180.0f;
}

static Vector3f reflect(const Vector3f &v, const Vector3f &n) {
    return v - n * dot(v,n)*2.0f;
}

// eta is n1/n2 which is the ratio of the prime medium's ior and the second  medium's ior
static Vector3f refract(const Vector3f &v, const Vector3f &n, const float eta) {
    Vector3f R;
    float k = 1.0f - eta * eta * (1.0f - dot(n, v) * dot(n, v));
    if (k < 0.0f)
        R = Vector3f(0.0f);       // or genDType(0.0)
    else
        R = v * eta - n * (eta * dot(n, v) + sqrtf(k));

    return R;
}

static const Vector3f GLOBAL_UP = Vector3f(0.0f, 0.0f, 1.0f);

struct Vector4f {
    union {
        struct { float x,y,z,w; };
        struct { float r,g,b,a; };
    };

    Vector4f() = default;
    Vector4f(float vx, float vy, float vz, float vw) : x(vx), y(vy), z(vz), w(vw) {}
    Vector4f(float v): x(v), y(v), z(v), w(v) {}
};

static Vector4f operator*(const Vector4f &a,const Vector4f &b)
{
    return {a.x*b.x, a.y*b.y, a.z*b.z, a.w*b.w};
}

static Vector4f operator+(const Vector4f &a,const Vector4f &b)
{
    return {a.x+b.x, a.y+b.y, a.z+b.z, a.w+b.w};
}

static Vector4f operator-(const Vector4f &a,const Vector4f &b)
{
    return {a.x-b.x, a.y-b.y, a.z-b.z, a.w-b.w};
}

struct Vector4b {
    union {
        struct { uint8_t x,y,z,w; };
        struct { uint8_t r,g,b,a; };
    };
};

static float lerp(float a, float b, float t)
{
    return (1.0f - t) * a + t * b;
}

static Vector4f lerp(Vector4f a, Vector4f b, Vector4f t)
{
    return (1.0f - t) * a + t * b;
}

static uint64_t FNV1A(uint64_t x)
{
    static constexpr uint64_t FNV_prime = 0x00000100000001B3;
    static constexpr uint64_t FNV_offset_basis = 0xcbf29ce484222325;

    uint64_t hash = FNV_offset_basis;
    uint8_t *data = reinterpret_cast<uint8_t *>(&x);

    for (int i = 0; i < 8; ++i) {
        hash = (hash & ~0xff) | ((hash & 0xff) ^ data[i]);
        hash *= FNV_prime;
    }

    return hash;
}

static uint64_t RandInt()
{
    static uint64_t globalSeed = 0;
    return FNV1A(globalSeed++);
}

// For smaller input rangers like audio tick or 0-1 UVs use these...
#define HASHSCALE1 443.8975f


static float RandFloat(float p)
{
    Vector3f p3  = Vector3f(fmodf(p * HASHSCALE1, 1.0f));
    p3 += dot(p3, Vector3f(p3.y,p3.z,p3.x) + 19.19f);
    return fmodf((p3.x + p3.y) * p3.z, 1.0f);
}

static float RandUniform()
{
    static float globalSeed = 0.0f;
    return (float)(RandInt() % UINT64_MAX) / (float)UINT64_MAX;
//    return RandFloat(globalSeed++);
}

static inline float RandFloatInRange(float x0, float x1)
{
    float len = fabsf(x1-x0);
//    return ((float)(RandInt() % UINT16_MAX) / (float)UINT16_MAX) * len + x0;
    return RandUniform() * len + x0;
}

template <typename T>
static T Min(const T& x, const T& y) { return x < y ? x : y; }

template <typename T>
static T Max(const T& x, const T& y) { return x > y ? x : y; }

static Vector3f Max(const Vector3f& x, const Vector3f& y)
{
    return {
        Max(x[0],y[0]),
        Max(x[1],y[1]),
        Max(x[2],y[2]),
        };
}
static Vector3f Min(const Vector3f& x, const Vector3f& y)
{
    return {
        Min(x[0],y[0]),
        Min(x[1],y[1]),
        Min(x[2],y[2]),
        };
}

static float clamp(float const& x, float const& a, float const& b) {
    return Min(Max(x, a), b);
}
static Vector3f saturate(const Vector3f &v) {
    return Vector3f(clamp(v.x, 0.0f, 1.0f), clamp(v.y, 0.0f, 1.0f), clamp(v.z, 0.0f, 1.0f));
}

struct Bounds3f {
    Vector3f v0, v1;

    Bounds3f() = default;
    Bounds3f(Vector3f mn, Vector3f mx) : v0(mn), v1(mx) { }

    inline bool contains(Point3f p) const {
        return (p.x >= v0.x && p.x <= v1.x
                && p.y >= v0.y && p.y <= v1.y
                && p.z >= v0.z && p.z <= v1.z
        );
    }

    inline Vector3f Dim() const {
        return {
            fabsf(v1.x - v0.x),
            fabsf(v1.y - v0.y),
            fabsf(v1.z - v0.z),
        };
    }

    inline float Dist(const Point3f &p) const {
        Vector3f v = p - (v0+v1)*0.5f;
        Vector3f b = (v1-v0)*0.5f;
        Vector3f q = Vector3f(fabsf(v.x), fabsf(v.y), fabsf(v.z)) - b;
        return length(Max(q,0.0f)) + Min(Max(q.x,Max(q.y,q.z)),0.0f);
    }

    inline float Dist2(const Point3f &p) const {
        Vector3f v = p - (v0+v1)*0.5f;
        Vector3f b = (v1-v0)*0.5f;
        Vector3f q = Vector3f(fabsf(v.x), fabsf(v.y), fabsf(v.z)) - b;
        return length2(Max(q,0.0f)) + Min(Max(q.x,Max(q.y,q.z)),0.0f);
    }
};
