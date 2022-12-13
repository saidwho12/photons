#ifndef BVH_H
#define BVH_H

#include <vector>
#include <algorithm>
#include "shape.h"
#include "mesh.h"

#define BVH_TRIANGLES_PER_LEAF 8

struct Vertex {
    Vector3f position;
    Vector3f normal;
    Vector2f uv;
    Vector3f tangent;
    Vector3f bitangent;
};

enum BvhAxis {
    BVH_X_AXIS_SPLIT = 0,
    BVH_Y_AXIS_SPLIT = 1,
    BVH_Z_AXIS_SPLIT = 2,
    BVH_AXIS_COUNT = 3
};

struct BvhNode {
    Bounds3f bounds;
    BvhNode *parent;
    BvhNode *child1, *child2;
    BvhAxis axis;
    uint32_t faces[BVH_TRIANGLES_PER_LEAF];
    bool is_leaf;
    size_t face_count;

    BvhNode(BvhNode *n = nullptr) : parent(n), is_leaf(false), child1(nullptr), child2(nullptr) {}
};

struct BvhFace {
    uint32_t e0,e1,e2;
};

struct BvhTree {
    std::vector<Bounds3f> face_bounds;
    std::vector<BvhFace> mesh_faces;
    std::vector<Vector3f> centroids;
    std::vector<Vertex> vertices;

    BvhNode *root;

    BvhTree() : root(nullptr) {}
    ~BvhTree() {}

    BvhNode *BuildNode(BvhNode *parent, BvhAxis axis, std::vector<uint32_t>& faces);
    void BuildBvh(ply_mesh& mesh);

    bool RayTraceNode(BvhNode *node, const Ray& ray, Hit *hit);
    bool RayTrace(const Ray& ray, Hit *hit);
};

struct MeshInstance : Shape {
    BvhTree *bvh;
    MeshInstance(BvhTree *mesh, Transform tf) : bvh(mesh),
    Shape(tf,SHAPE_TYPE_BVH_TRIANGLE_MESH) {}
    
    Bounds3f ObjectBound() const override {
        if (bvh->root != nullptr) {
            return bvh->root->bounds;
        }

        return Bounds3f{};
    }

    bool Intersect(const Ray &ray, Hit *hit) const override {
        Ray r2 = objectToWorld.AdjustRay(ray);
        if (!bvh->RayTrace(r2,hit)) {
            return false;
        }

        return true;
    }

    float Area() const override {
        return 0.0f;
    }
};

#endif // BVH_H