#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <iostream>
using namespace std;

// TODO: implement this class and add more fields as necessary, finish?
class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle( const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
		this->a = a;
		this->b = b;
		this->c = c;
		vertices[0] = a;
		vertices[1] = b;
		vertices[2] = c;
		normal = Vector3f::cross(a-b, a-c).normalized();
	}

	bool intersect( const Ray& ray,  Hit& hit , float tmin) override {

		// 光线与三角形所在面是否相交
		float d = Vector3f::dot(a,normal);
		float t_ = ( d - Vector3f::dot(ray.getOrigin(), normal)) / Vector3f::dot(ray.getDirection(), normal);
		if(t_ < 0) return false; // 光线与三角形所在平面不相交
		Vector3f p = ray.pointAtParameter(t_); // 交点

		// 判断交点是否在三角形内部
		Vector3f e1 = a - b;
		Vector3f e2 = a - c;
		Vector3f s = a - ray.getOrigin();
		float t = Vector3f::dot(Vector3f::cross(e1, e2), s) / Vector3f::dot(Vector3f::cross(e1, e2), ray.getDirection());
		float beta = Vector3f::dot(Vector3f::cross(s, e2), ray.getDirection()) / Vector3f::dot(Vector3f::cross(e1, e2), ray.getDirection());
		float gama = Vector3f::dot(Vector3f::cross(e1, s), ray.getDirection()) / Vector3f::dot(Vector3f::cross(e1, e2), ray.getDirection());
		// 交点不在三角形内部
		if(beta < 0 || gama < 0 || beta + gama > 1) return false;
		if(t < hit.getT() && t > tmin) {
			hit.set(t, material, normal);
			return true;
		}
        return false;
	}

	Vector3f normal;
	Vector3f vertices[3];

protected:
	Vector3f a, b, c;
};

#endif //TRIANGLE_H
