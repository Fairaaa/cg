#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement functions and add more fields as necessary finish?

class Sphere : public Object3D {
public:

    Sphere() {
        // unit ball at the center
        center = Vector3f(0, 0, 0);
        radius = 1;
    }

    Sphere(const Vector3f &center, float radius, Material *material) : Object3D(material) {
        // 
        this->center = center;
        this->radius = radius;
    }

    ~Sphere() override = default;

    // 光线r h返回求交结果 tmin返回最近交点
    bool intersect(const Ray &r, Hit &h, float tmin) override {
        // 判断光线是否和球相交
        // 光线方程 P = O + t * D
        // 光线和球心距离
        // 光心为c 球心为o 球心距离光线最近的点d
        Vector3f ray_dir = r.getDirection().normalized();
        float ray_d = Vector3f::dot(r.getDirection(), ray_dir);
        // 是否发生变换
        bool flag = (ray_d != 1);
        Vector3f co = center - r.getOrigin();
        float dis_cd = Vector3f::dot(co, ray_dir); 
        float dis_od = Vector3f::dot(co, co) - dis_cd * dis_cd; // od距离的平方
        // 距离大于半径 不相交
        if (dis_od > radius * radius) {
            return false;
        }
        // 找到最近交点
        float dis_h = sqrt(radius * radius - dis_od);
        float t;
        if(Vector3f::dot(co, co) < radius * radius)
        {
            t = dis_cd + dis_h;
        }
        else
        {
            t = dis_cd - dis_h;
        }
        // 交点
        if(flag)
        {
            t = t / ray_d;
        }
        if(t > tmin && t < h.getT())
        {
            Vector3f normal = (r.pointAtParameter(t) - center).normalized();
            h.set(t, material, normal);
            return true;
        }
        return false;
    }

protected:
    Vector3f center;
    float radius;
};


#endif
