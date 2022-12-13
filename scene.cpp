
 #include "scene.h"

 Scene::Scene(){

//    walls[0] = *new Rectangle(*new Vertex(552.8f, 0, 0), *new Vertex(0, 0, 0), *new Vertex(0, 0, 559.2f), *new Vertex(549.6f, 0, 559.2f), *new Colour(1.0f, 1.0f, 1.0f));
//    walls[1] = *new Rectangle(*new Vertex(556.0f, 548.8f, 0), *new Vertex(556.0f, 548.8f, 559.2f), *new Vertex(0, 548.8f, 559.2f), *new Vertex(0, 548.8f, 0), *new Colour(1.0f, 1.0f, 1.0f));
//    walls[2] = *new Rectangle(*new Vertex(549.6f, 0, 559.2f), *new Vertex(0, 0, 559.2f), *new Vertex(0, 548.8f, 559.2f), *new Vertex(556.0f, 548.8f, 559.2f), *new Colour(1.0f, 1.0f, 1.0f));
//    walls[3] = *new Rectangle(*new Vertex(0, 0, 559.2f), *new Vertex(0, 0, 0), *new Vertex(0, 548.8f, 0), *new Vertex(0, 548.8f, 559.2f), *new Colour(0.0f, 1.0f, 0.0f));
//    walls[4] = *new Rectangle(*new Vertex(552.8f, 0, 0), *new Vertex(549.6f, 0, 559.2f), *new Vertex(556.0f, 548.8f, 559.2f), *new Vertex(556.0f, 548.8f, 0), *new Colour(1.0f, 0.0f, 0.0f));


 }


//bool Scene::intersect(Ray ray, Hit *hit)
//{
//    GeometryType geometryType = NO_GEOMETRY;
//    ObjectId objectId = 0;
//
//    hit->t = std::numeric_limits<float>::infinity();
//
//    for (uint32_t i = 0; i < sphereCount; ++i) {
//        for (int i = 0; i < 5; i++){
//
//            if (hit.flag == false){
//                intersection_rectangle(ray, hit, walls[i]);
//            }
//        }
//    }
// }

int Scene::materialIdCounter = 0;
int Scene::textureIdCounter = 0;

int Scene::AddMaterial(Material m)
{
    int id = NextMaterialID();
    materials.insert(std::pair<int, Material>(id, m));//[NextMaterialID()] = m;
    return id;
}

void Scene::AddPrimitive(Shape *shape, const Material &m)
{
    int matid = AddMaterial(m);
    primitives.emplace_back(std::shared_ptr<Shape>(shape), matid );
}

void Scene::AddPointLight(const PointLight &light) {
    pointLights.push_back(light);
}

Bounds3f Scene::WorldBound() const
{
    Vector3f v0 {FLT_MAX}, v1 {FLT_MIN};

    for (const GeometricPrimitive &primitive: primitives) {
        Bounds3f bound = primitive.WorldBound();
        v0 = Min(v0, bound.v0);
        v1 = Max(v1, bound.v1);
    }

    return {v0,v1};
}


const Material *Scene::GetMaterial(int id) const
{
    return &materials.at(id);
}

 int Scene::AddTexture(Texture texture) {
    int id = NextTextureID();
    textures.insert(std::pair<int, Texture>(id, texture));//[NextMaterialID()] = m;
    return id;
 }

 Texture *Scene::GetTexture(int textureId) const {
     return const_cast<Texture *>(&textures.at(textureId));
 }

 Vector3f Scene::SampleEnv(const Vector3f &v) const {
    // For now, direction is ignored as environment maps are not implemented
    return skyColour;
 }
