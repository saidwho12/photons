

#pragma once

#include "math.h"
#include <iostream>

#include "ray.h"

struct Transform {
    Vector3f translation;
    Vector3f scale;

    Transform() : translation(0.0f), scale(1.0f) {}
    Transform(const Vector3f &position) : translation(position), scale(1.0f) {}

    Ray AdjustRay(Ray r) const {
        r.o -= translation;
        return r;
    }
};
