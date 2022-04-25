

#pragma once
#include "math.h"

#define RAY_T_MIN 0.0f
#define RAY_T_MAX 1000.0f
#define RAY_EPSILON 0.0001f

class Ray {
public:
	Vector3f o;
	Vector3f d;
	float t1, t2;
	float ior;

	Ray(float tmin = RAY_T_MIN, float tmax = RAY_T_MAX) : t1(tmin), t2(tmax), ior(1.0f)
	{
	}

	Ray(Point3f ro, Vector3f rd, float tmin = RAY_T_MIN, float tmax = RAY_T_MAX)
	: o(ro), d(rd), t1(tmin), t2(tmax), ior(1.0f)
	{}
};
