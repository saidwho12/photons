

#pragma once

#include "math.h"
#include <iostream>

using namespace std;

struct Transform {
    Vector3f translation;
    Vector3f scale;

    Transform() : translation(0.0f), scale(1.0f) {}

    Transform(const Vector3f &position) : translation(position), scale(1.0f) {}
};
