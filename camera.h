#include "transform.h"
#include "ray.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846  /* pi */
#endif

class Camera {
public:

    Point3f position;
    Transform projectionMatrix;
    float horizontalFov;
    float bias;
    float maxDepth;
    float aspectRatio;
    Point3f cameraLookAt;

    Vector3f f,u,r; // forward,up,right

    Camera(){

    }

    Camera(Point3f eye, Point3f center, float aspect, float fov) : position(eye), aspectRatio(aspect) {
//        projectionMatrix = lookAt(eye, center);
//        projectionMatrix.multVecMatrix(*new Point3f(0, 0, 0), position);

        f = normalise(center - eye);
        r = normalise(cross(f, GLOBAL_UP));
        u = cross(r, f);

        horizontalFov = Radians(fov);
        bias = 0.0f;
        maxDepth = 1000.0f;
        aspectRatio = aspect;
        cameraLookAt = center;
    }

//    void projectPixels(int j, int i, float &x, float &y) {
//        x = (2 * (i + 0.5) / width - 1) * imageAspectRatio * scale;
//        y = (1 - 2 * (j + 0.5) / height) * scale;
//
//    }

//    Transform lookAt(Point3f from, Point3f to){
//
//        Vector3f f;
//
//        f.x = from.x - to.x;
//        f.y = from.y - to.y;
//        f.z = from.z - to.z;
//
//        f.normalise();
//
//        Vector3f temp;
//        temp.x = 0;
//        temp.y = 1;
//        temp.z = 0;
//        temp.normalise();
//
//        Vector3f r = temp.cross(f);
//
//        Vector3f u = f.cross(u);
//
//        return Transform(r.x, u.x, f.x, from.x, r.y, u.y, f.y, from.y, r.z, u.z, f.z, from.z, 0, 0, 0, 1);
//
//    }

    // Gets ray at current UV position within viewport if it was shot out from the center point of the camera.
    // This style of function will be useful for DOF/MSAA
    Ray GetRayAtPosition(const Vector2f &uv) const {
        Ray ray;
        ray.o = position;

        Vector2f p = (uv * 2.0f) - 1.0f;

        float halfWidth = tanf(horizontalFov / 2.0f);
        float halfHeight = halfWidth / aspectRatio;

        ray.d = normalise(
                f + r * halfWidth * p.x + u * halfHeight * p.y);

        return ray;
    }
};

