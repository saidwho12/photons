#include "bvh.h"

bool IntersectTriangle(const Ray& ray, Vertex v0, Vertex v1, Vertex v2, Hit *hit) {

    float epsilon = 1.e-8;

    Vector3f e1 = v1.position - v0.position;
    Vector3f e2 = v2.position - v0.position;

    hit->normal = normalise(cross(e1,e2));
    // e1.cross(e2, hit->normal);
    // hit->normal.normalise();

    Vector3f pvec = cross(ray.d,e2);
    // ray.d.cross(e2, pvec);

    float det = dot(e1,pvec);

    if (fabs(det) < epsilon) {
        return false;
    }

    float inv_det = 1.0f / det;

    Vector3f tvec = ray.o - v0.position;

    float u = dot(tvec,pvec) * inv_det;

    if (u < 0 || u > 1) {
        return false;
    }

    Vector3f qvec = cross(tvec,e1);

    float v = dot(ray.d,qvec) * inv_det;
    if (v < 0 || u + v > 1) {
        return false;
    }

    float w = 1.0f - u - v;

    float d = -hit->normal.dot(v0.position);
    hit->t = -(hit->normal.dot(ray.o) + d) / hit->normal.dot(ray.d);

    if (hit->t < 0.0f)
    {
        return false;
    }

    // Calculate point of intersection
    hit->position = ray.o + ray.d * hit->t;
    // hit->what = this;

    static bool smoothing = true;
    if (smoothing)
    {
        hit->normal = w * v0.normal + u * v1.normal + v * v2.normal;
    }

    if (dot(ray.d,hit->normal) > 0.0f)
    {
        hit->normal.negate();
    }

    return true;
    // float epsilon = 1e-8;

    // Vector3f e1 = v1.position - v0.position;
    // Vector3f e2 = v2.position - v0.position;

    // hit->normal = (e1.cross(e2)).normalised();

    // Vector3f pvec = ray.d.cross(e2);

    // float det = e1.dot(pvec);

    // if (fabs(det) < epsilon) {
    //     return false;
    // }

    // float inv_det = 1 / det;

    // Vector3f tvec = ray.o - v0.position;

    // float u = tvec.dot(pvec) * inv_det;

    // if (u < 0 || u > 1) {
    //     return false;
    // }

    // Vector3f qvec = tvec.cross(e1);

    // float v = ray.d.dot(qvec) * inv_det;
    // if (v < 0 || u + v > 1) {
    //     return false;
    // }

    // float w = 1.0f - u - v;

    // float d = -hit->normal.dot(v0.position);
    // hit->t = -(hit->normal.dot(ray.o) + d) / hit->normal.dot(ray.d);

    // if (hit->t < 0)
    // {
    //     return false;
    // }

    // // Calculate point of intersection
    // hit->position = ray.o + hit->t * ray.d;
    // // hit->primitive = dynamic_cast<const Primitive*>(this);

    // // if (smoothing)
    // // {
    // //     hit.normal = w * v0.normal + u * v1.normal + v * v2.normal;
    // // }

    // if (ray.d.dot(hit->normal) > 0)
    // {
    //     hit->normal.negate();
    // }

    // return true;
}

BvhNode *BvhTree::BuildNode(BvhNode *parent, BvhAxis axis, std::vector<uint32_t>& faces) {
    if (!faces.size()) {
        return nullptr; // no triangles, no node to be created for empty space
    }

    // allocate new node
    BvhNode *node = new BvhNode(parent);
    node->axis = axis;

    // compute new node's bounds
    node->bounds.v0 = Vector3f(FLT_MAX);
    node->bounds.v1 = Vector3f(FLT_MIN);

    for (auto &f : faces) {
        node->bounds.v0 =  V3Min(node->bounds.v0, face_bounds[f].v0);
        node->bounds.v1 =  V3Max(node->bounds.v1, face_bounds[f].v1);
    }

    // printf("(%f,%f,%f) -> (%f,%f,%f)\n",node->bounds.v0.x,node->bounds.v0.y,node->bounds.v0.z,
    // node->bounds.v1.x,node->bounds.v1.y,node->bounds.v1.z);
    
    if (faces.size() <= BVH_TRIANGLES_PER_LEAF) {
        // this is a leaf node, terminate recursion
        node->is_leaf = true;
        memcpy(node->faces, &faces[0], sizeof(uint32_t) * faces.size());
        node->face_count = faces.size();
        // printf("Reached leaf %d!\n", faces.size());
        return node;
    }

    // sort faces on picked axis
    size_t face_count = faces.size();
    std::sort(faces.begin(), faces.end(), [&](uint32_t &e1, uint32_t &e2) -> int {
            Vector3f c1 = centroids[e1],c2 = centroids[e2];
            return c1[axis] > c2[axis];
        }
    );

    float split_plane = centroids[faces[face_count/2]][axis];
    std::vector<uint32_t> child1_faces, child2_faces;

    for (auto &f : faces) {
        Vector3f center = centroids[f];
        if (center[axis] <= split_plane) {
            child1_faces.push_back(f);
        } else {
            child2_faces.push_back(f);
        }
    }

    //printf("Building BVH node with %d polys (l:%d, r:%d) with splitting plane at %f!\n", faces.size(), child1_faces.size(), child2_faces.size(),split_plane);

    BvhAxis children_axis = (BvhAxis)((axis + 1) % BVH_AXIS_COUNT);
    
    node->child1 = BuildNode(node,children_axis,child1_faces);
    node->child2 = BuildNode(node,children_axis,child2_faces);

    return node;
}

void BvhTree::BuildBvh(ply_mesh& mesh) {
    auto &e_vertex = mesh.elements[0];
    auto &e_face = mesh.elements[1];

    ply_property *x_prop = e_vertex.find_property("x");
    ply_property *y_prop = e_vertex.find_property("y");
    ply_property *z_prop = e_vertex.find_property("z");
    ply_property *nx_prop = e_vertex.find_property("nx");
    ply_property *ny_prop = e_vertex.find_property("ny");
    ply_property *nz_prop = e_vertex.find_property("nz");
    ply_property *s_prop = e_vertex.find_property("s");
    ply_property *t_prop = e_vertex.find_property("t");
    float *x_arr = reinterpret_cast<float*>(&x_prop->storage.data[0]);
    float *y_arr = reinterpret_cast<float*>(&y_prop->storage.data[0]);
    float *z_arr = reinterpret_cast<float*>(&z_prop->storage.data[0]);
    float *nx_data = nullptr, *ny_data = nullptr, *nz_data = nullptr;
    float *s_arr = nullptr, *t_arr = nullptr;
    if (s_prop != nullptr) s_arr = reinterpret_cast<float*>(&s_prop->storage.data[0]);
    if (t_prop != nullptr) t_arr = reinterpret_cast<float*>(&t_prop->storage.data[0]);
    if (nx_prop != nullptr) nx_data = reinterpret_cast<float*>(&nx_prop->storage.data[0]);
    if (ny_prop != nullptr) ny_data = reinterpret_cast<float*>(&ny_prop->storage.data[0]);
    if (nz_prop != nullptr) nz_data = reinterpret_cast<float*>(&nz_prop->storage.data[0]);
    
    for (size_t i = 0; i < e_vertex.count; ++i) {
        Vertex v;
        v.position = Vector3f(x_arr[i],y_arr[i],z_arr[i]);
        if (s_arr != nullptr && t_arr != nullptr) v.uv = Vector2f(s_arr[i],t_arr[i]);
        if (nx_data != nullptr && ny_data != nullptr && nz_data != nullptr) {
            v.normal = Vector3f(nx_data[i],ny_data[i],nz_data[i]);
        }

        vertices.push_back(v);
    }

    const ply_property* index_prop = e_face.find_property("vertex_indices");
    const uint32_t *face_data = reinterpret_cast<const uint32_t*>(&index_prop->storage.data[0]);
    for (size_t i = 0; i < e_face.count; ++i) {
        BvhFace f;
        f.e0 = face_data[i*3+0];
        f.e1 = face_data[i*3+1];
        f.e2 = face_data[i*3+2];
        mesh_faces.push_back(f);
        
        Vertex v0 = vertices[f.e0];
        Vertex v1 = vertices[f.e1];
        Vertex v2 = vertices[f.e2];

        Vector3f centroid = (v0.position + v1.position + v2.position) / 3.0f;
        centroids.push_back(centroid);

        Bounds3f bounds;
        bounds.v0 = V3Min(V3Min(v0.position,v1.position),v2.position);
        bounds.v1 = V3Max(V3Max(v0.position,v1.position),v2.position);
        face_bounds.push_back(bounds);
    }

    std::vector<uint32_t> node_faces;
    for (size_t i = 0; i < e_face.count; ++i) {
        node_faces.push_back(i);
    }

    root = BuildNode(nullptr,BVH_X_AXIS_SPLIT,node_faces);
}

bool IntersectBounds(const Bounds3f &b, const Ray &r) {
    Vector3f n_inv = Vector3f(1.0f / r.d.x, 1.0f / r.d.y, 1.0f / r.d.z);
    Vector3f lb = b.v0;
    Vector3f rt = b.v1;
    float t;

    // lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
    // r.org is origin of ray
    float t1 = (lb.x - r.o.x)*n_inv.x;
    float t2 = (rt.x - r.o.x)*n_inv.x;
    float t3 = (lb.y - r.o.y)*n_inv.y;
    float t4 = (rt.y - r.o.y)*n_inv.y;
    float t5 = (lb.z - r.o.z)*n_inv.z;
    float t6 = (rt.z - r.o.z)*n_inv.z;

    float tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
    float tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
    if (tmax < 0.0f)
    {
        t = tmax;
        return false;
    }

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax)
    {
        t = tmax;
        return false;
    }

    t = tmin;
    return true;
}

bool BvhTree::RayTraceNode(BvhNode *node, const Ray& ray, Hit *hit) {
    // check against node AABB
    if (node == nullptr) return false;

    if (!IntersectBounds(node->bounds,ray)) {
        return false;
    }

    if (node->is_leaf) {
        // hit a leaf node, check intersection against triangles
        float nearestT = ray.t2; // start with maximum t value of ray
        Hit nearestHit;
        bool hitAny = false;
        
        for (int i = 0; i < node->face_count; ++i) {
            BvhFace f = mesh_faces[node->faces[i]];
            Vertex v0 = vertices[f.e0];
            Vertex v1 = vertices[f.e1];
            Vertex v2 = vertices[f.e2];
            Hit currHit{};
            if (IntersectTriangle(ray,v0,v1,v2,&currHit)) {
                if (currHit.t >= ray.t1 && currHit.t < nearestT) {
                    nearestT = currHit.t, nearestHit = currHit;
                    hitAny = true;
                }
            }
        }

        if (hitAny) *hit = nearestHit;
        return hitAny;
    }
    
    {
        float nearestT = ray.t2; // start with maximum t value of ray
        Hit nearestHit;
        bool hitAny = false;
        { Hit currHit{};
            if (RayTraceNode(node->child1,ray,&currHit)) {
                if (currHit.t >= ray.t1 && currHit.t < nearestT) {
                    nearestT = currHit.t, nearestHit = currHit;
                    hitAny = true;
                }
            }
        }

        { Hit currHit{};
            if (RayTraceNode(node->child2,ray,&currHit)) {
                if (currHit.t >= ray.t1 && currHit.t < nearestT) {
                    nearestT = currHit.t, nearestHit = currHit;
                    hitAny = true;
                }
            }
        }

        if (hitAny) *hit = nearestHit;
        return hitAny;
    }
}

bool BvhTree::RayTrace(const Ray& ray, Hit *hit) {
    return RayTraceNode(root,ray,hit);
}
