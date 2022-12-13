#include "lighting.h"

Ray PointLight::RandomRay() const {
    // Generates random direction using spherical coordinates theta and phi
    Ray ray;
    ray.o = position;

    float theta = RandFloatInRange(0.0f, 1.0f*M_PI);
    float phi = RandFloatInRange(0.0f, 2.0f*M_PI);

    float cosTheta = cosf(theta);
    float cosPhi = cosf(phi);
    float sinTheta = sinf(theta);
    float sinPhi = sinf(phi);

    ray.d = Vector3f(cosTheta*sinPhi,sinPhi*sinTheta,cosPhi);

    return ray;
}

float PointLight::PowerFalloff(const Point3f &worldPoint) const {
    // Geometric falloff for point lights is inversely proportional
    // to area of sphere at distance. See inverse square law: https://en.wikipedia.org/wiki/Inverse-square_law
    float dist = length(worldPoint - position);
    return power / (4.0f * M_PI * powf(dist,2.0f));
}