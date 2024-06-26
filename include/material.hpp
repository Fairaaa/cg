#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include <iostream>


enum Type{DIFF, SPEC, REFR};

// TODO: Implement Shade function that computes Phong introduced in class.
class Material {
public:

    // explicit Material(const Vector3f &d_color, const Vector3f &s_color = Vector3f::ZERO, float s = 0) :
    //         diffuseColor(d_color), specularColor(s_color), shininess(s) {
    // }

    explicit Material(const Vector3f &d_color, const Vector3f &s_color, float s, Type r, const Vector3f &e) :
            diffuseColor(d_color), specularColor(s_color), shininess(s), refl(r), emission(e) {
    }

    virtual ~Material() = default;

    virtual Vector3f getDiffuseColor() const {
        return diffuseColor;
    }
    virtual Type getRefl() const {
        return refl;
    }
    virtual Vector3f getEmission() const {
        return emission;
    }


    Vector3f Shade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor) {
        Vector3f shaded = Vector3f::ZERO;

        // 相交处法向量
        Vector3f N = hit.getNormal();
        // 从相交处p指向光源的单位向量
        Vector3f L = dirToLight.normalized();
        // 
        Vector3f V = - ray.getDirection();
        // 反射光线方向
        Vector3f R = 2 * Vector3f::dot(N, L) * N - L;

        // phong模型：漫反射+镜面反射
        shaded = lightColor * (diffuseColor * std::max(0.0f, Vector3f::dot(N, L)) + 
                               specularColor * std::pow(std::max(0.0f, Vector3f::dot(R, V)), shininess));

        return shaded;
    }

protected:
    Vector3f diffuseColor;
    Vector3f specularColor;
    float shininess;
    Type refl;
    Vector3f emission;
};

// class Lambertian : public Material {
//     public:
//         Lambertian(const Vector3f &d_color) : tex(make_shared<solid_color>(d_color)) {}
//         Lambertian(shared_ptr<texture> t) : tex(t) {}

//         bool scatter(const Ray &ray, const Hit &hit, Vector3f &attenuation, Ray &scattered) const override {
//             Vector3f target = hit.getNormal() + Vector3f::random3f_unit(); // 随机生成一个单位向量
            
//             if(Vector3f::dot(target, hit.getNormal()) < 0.0)
//             {
//                 target = hit.getNormal();
//             }
            
//             scattered = Ray(hit.getPoint(), target);
//             attenuation = tex->value(0, 0, hit.getPoint());
//             return true;
//         }
//     private:
//         shared_ptr<texture> tex;
// };


#endif // MATERIAL_H
