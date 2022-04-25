#pragma once

#include <cstdint>
#include "ray.h"

struct SpotLight {
    Vector3f o, d;
    float theta;

    Vector3f color;
    float intensity;

    SpotLight(Vector3f co, Vector3f cd) : o(co), d(cd) {}
    ~SpotLight() {}
};

struct PointLight {
    Point3f position;
    Vector3f albedo;
    float power;

    PointLight(Point3f pos, Vector3f col, float p) : position(pos), albedo(col), power(p) {}

    Ray RandomRay() const;
    float Pdf() const;
    float PowerFalloff(const Point3f &worldPoint) const;
};