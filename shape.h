#pragma once

#include "ray.h"
#include "math.h"
#include "material.h"
#include "texture.h"
#include "transform.h"
#include <limits>
#include <cstdint>
#include <memory>
#include <utility>

struct Hit;

// Enumeration of geometry types, values are unique, but also can be used as flags for sorting
// or debugging for example
enum ShapeType : uint32_t {
    NO_GEOMETRY = 0,
    SHAPE_TYPE_INFINITE_PLANE = 0x00000001,
    SHAPE_TYPE_PLANE = 0x00000002,
    SHAPE_TYPE_SPHERE = 0x00000004,
    SHAPE_TYPE_DISK = 0x00000008,
    SHAPE_TYPE_TRIANGLE_MESH = 0x00000010,
    SHAPE_TYPE_AXIS_ALIGNED_BOUNDING_BOX = 0x00000020,
    SHAPE_TYPE_ORIENTED_BOUNDING_BOX = 0x00000040,
};

// Abstract shape class provides interface for intersection, bounding volume computation, and more
struct Shape {
    Shape(const Transform &objectToWorld, const ShapeType &shapeType)
    : objectToWorld(objectToWorld), shapeType(shapeType)
    {}

    virtual ~Shape() {}

    virtual Bounds3f ObjectBound() const = 0;
    virtual Bounds3f WorldBound() const;
    virtual bool Intersect(const Ray &ray, Hit *hit) const = 0;
    virtual float Area() const = 0;

    Transform objectToWorld;
    ShapeType shapeType;
};

struct Sphere : Shape {
    float radius;
    Sphere(Point3f c, float r) : Shape(Transform(c), SHAPE_TYPE_SPHERE), radius(r) {}

    Bounds3f ObjectBound() const override;
    bool Intersect(const Ray &ray, Hit *hit) const override;
    float Area() const override;
};

struct Disk : Shape {
    // normal direction, this should really be using the Transform class,
    // but rotation is not trivial to implement
    Vector3f n;
    float radius;

    Disk(Point3f o, Vector3f v, float r)
    : Shape(Transform(o), SHAPE_TYPE_DISK), n(v), radius(r)
    {}

    Bounds3f ObjectBound() const override;
    bool Intersect(const Ray &ray, Hit *hit) const override;
    float Area() const override;
};

struct Primitive {
    virtual ~Primitive() {}
    virtual Bounds3f WorldBound() const = 0;
    virtual bool Intersect(const Ray &ray, Hit *hit) const = 0;
    virtual int GetMaterialID() const = 0;
};

struct GeometricPrimitive : Primitive {
    GeometricPrimitive(std::shared_ptr<Shape> s, int matid)
    : shape(std::move(s)), materialId(matid)
    { }

    Bounds3f WorldBound() const override;
    bool Intersect(const Ray &ray, Hit *hit) const override;
    int GetMaterialID() const override { return materialId; }

    const Shape *GetShapePointer() const { return shape.get(); }
private:
    std::shared_ptr<Shape> shape;
    int materialId;
};

// Hit record stores intersection information
struct Hit {
    bool frontFace;
    float t;
    Point3f position;
    Vector3f normal;
    bool hasUv;
    Vector2f uv;
    const Primitive *primitive;
};
