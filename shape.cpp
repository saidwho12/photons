#include "shape.h"

bool Sphere::Intersect(const Ray &ray, Hit *hit) const
{
    Vector3f v = ray.o - objectToWorld.translation;
    float b = 2.0f * dot(ray.d, v), c = dot(v, v) - radius * radius, delta = b * b - 4.0f * c;
    float t = -1.0f;

    if (delta == 0.0f && -b >= 0.0f) {
        // edge intersection
        t = -b*0.5f;
        hit->frontFace = true;
    } else if (delta > 0.0f) {
        // two solutions
        float h = sqrtf(delta);
        float t0 = -b+h, t1 = -b-h;
        float tmin = Min(t0, t1), tmax = Max(t0, t1);

        if (tmin > 0.0f) {
            hit->frontFace = true;
            t = tmin*0.5f;
        } else if (tmax > 0.0f) {
            hit->frontFace = false;
            t =  tmax*0.5f;
        }
    }

    if (t >= 0.0f) {
        auto p = ray.o + ray.d*t;
        auto n = (p - objectToWorld.translation)/radius;
        hit->t = t;

        // cylindrical UV unwrap for sphere texturing
        // https://en.wikipedia.org/wiki/UV_mapping
        hit->uv = Vector2f(1.0f + atan2f(n.x, n.y)/M_PI,
                           0.5f - asinf(n.z)/M_PI);

        hit->hasUv = true;

        hit->normal = hit->frontFace ? n : -n;
        hit->position = p;
        return true;
    }

    return false;
}

Bounds3f Sphere::ObjectBound() const
{
    return Bounds3f(-radius, radius);
}

float Sphere::Area() const
{
    return 4.0f * M_PI * powf(radius, 2.0f);
}

Bounds3f Disk::ObjectBound() const
{
    return Bounds3f(-radius, radius);
}

float Disk::Area() const
{
    return M_PI * powf(radius, 2.0f);
}

bool Disk::Intersect(const Ray &ray, Hit *hit) const
{
        Vector3f p = objectToWorld.translation - ray.o;
        float nv = dot(ray.d, n), np = dot(n, p);
        float t = -1.0f;

        // Checks if ray is orthogonal to the plane the disk lies on
        if (std::fabs(nv) > std::numeric_limits<float>::min()) {
            float k = np/nv;  // distance to plane the disk lies on

            // Find hitpoint along plane
            Vector3f m = objectToWorld.translation - (ray.o + ray.d * k);
            float v = dot(m,m);

            // Check if projected hit point within disk
            if (v <= radius*radius) {
                t = k;
            }
        }

        if (t != -1.0f) {
            if (hit != nullptr) {
                hit->t = t;
                hit->normal = (np < 0.0f) ? n : -n;
                hit->position = ray.o + ray.d * t;
                hit->frontFace = true;
                hit->hasUv = false;
            }

            return true;
        }


        return false;
}

bool GeometricPrimitive::Intersect(const Ray &ray, Hit *hit) const
{
    if (!shape->Intersect(ray, hit))
        return false;

    if (hit != nullptr)
        hit->primitive = dynamic_cast<const Primitive *>(this);

    return true;
}

Bounds3f GeometricPrimitive::WorldBound() const {
    return shape->WorldBound();
}

//float IntersectDisk(const Disk& disk, const Ray &ray) {
//    Vector3f p = disk.c - ray.o, v = ray.d;
//    Vector3f n = disk.n;
//    float nv = Dot(v,n);
//
//    // Checks if ray is orthogonal to the plane the disk lies on
//    if (std::fabs(nv) > std::numeric_limits<float>::min()) {
//        float t = Dot(n,p)/nv;
//
//        Vector3f tangent = Normalise(Cross(n,Vector3f(n.y,n.z,n.x)));
//        Vector3f bitangent = Cross(n,tangent);
//
//        // Find hitpoint along plane
//        Vector3f m = disk.c - (ray.o + ray.d * t);
//
//        float x = tangent.dot(m);
//        float y = bitangent.dot(m);
//
//        // Check if projected hit point within disk
//        if (sqrt(x*x + y*y) <= disk.r) {
//            return t;
//        }
//    }
//
//    return -1.0f;
//}
Bounds3f Shape::WorldBound() const {
    auto bounds = ObjectBound();
    bounds.v0 += objectToWorld.translation;
    bounds.v1 += objectToWorld.translation;
    return bounds;
}