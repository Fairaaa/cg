#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions
// finish?

class Plane : public Object3D {
public:
    Plane() {
        normal = Vector3f(0, 1, 0);
        d = 0;
    }

    Plane(const Vector3f &normal, float d, Material *m) : Object3D(m) {
        this->normal = normal;
        this->d = d;
    }

    ~Plane() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        // 判断光线是否和平面相交
        float t = (d + Vector3f::dot(normal, r.getOrigin())) / Vector3f::dot(normal, r.getDirection());
        if(t > tmin && t < h.getT()) {
            h.set(t, material, normal);
            return true;
        }
        return false;
    }

protected:
    // 平面的表示：ax+by+cz=d the normal is (a,b,c)
    Vector3f normal;
    float d;

};

#endif //PLANE_H
		

