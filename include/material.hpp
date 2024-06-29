#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <string>

#include "stb_image.h"

using namespace std;


enum Type{DIFF, SPEC, REFR};


class Texture
{
public:
    unsigned char *data;
    int width, height;

    Texture(const char *filename)
    {
        data = stbi_load(filename, &width, &height, 0, 3);
        std::cout << "width: " << width << " height: " << height << std::endl;
    }
    ~Texture()
    {
        stbi_image_free(data);
    }
    Vector3f getColor(float u, float v)
    {
        if(data == nullptr) return Vector3f::ZERO;
        u = u - floor(u);
        v = v - floor(v);
        int x = u * width;
        int y = v * height;
        if(x < 0) x = 0;
        if(x >= width) x = width - 1;
        if(y < 0) y = 0;
        if(y >= height) y = height - 1;
        int index = 3 * (y * width + x);
        assert(index + 2 < width * height * 3);
        return Vector3f(data[index], data[index + 1], data[index + 2]) / 255.0f;
    }
};


// TODO: Implement Shade function that computes Phong introduced in class.
class Material {
public:

    bool isTexture = false;
    string texture_file;
    Texture *texture = nullptr;
    bool isLight = false;
    

    // explicit Material(const Vector3f &d_color, const Vector3f &s_color = Vector3f::ZERO, float s = 0) :
    //         diffuseColor(d_color), specularColor(s_color), shininess(s) {
    // }

    explicit Material(const Vector3f &d_color, Type r, const Vector3f &s = (0,0,0), float shininess = 0.0, const Vector3f &e = (0,0,0), float refraction = 1.0) :
            diffuseColor(d_color), refl(r), specularColor(s), shininess(shininess), emission(e),refraction(refraction) {
        if(e.x() > 0.0 || e.y() > 0.0 || e.z() > 0.0) isLight = true;
    }
    explicit Material(string texture) : texture_file(texture)
    {
        std::cout << "texture: " << texture << std::endl;
        isTexture = true;
        this->texture = new Texture(texture.c_str());
    }

    virtual ~Material() = default;

    virtual Vector3f getDiffuseColor() const {
        return diffuseColor;
    }

    virtual Vector3f getTexColor(Vector3f point) const {
        if(!isTexture) return Vector3f::ZERO;
        // point转化为（-1，-1,1）下半径为0.75的球的坐标
        // U 坐标对应球体表面的水平位置
        // V 坐标对应球体表面的垂直位置
        Vector3f center = Vector3f(0, 0, 0);
        float radius = 1.2;

        Vector3f p = point - center;
        float phi = atan2(p.z(), p.x());
        float theta = asin(p.y() / radius);
        float u = 1 - (phi + M_PI) / (2 * M_PI);
        float v = 1 - (theta + M_PI / 2) / M_PI;

        assert(u >= 0 && u <= 1);
        assert(v >= 0 && v <= 1);

        
        return texture->getColor(u, v);
    }


    virtual Type getRefl() const {
        return refl;
    }
    virtual Vector3f getEmission() const {
        return emission;
    }
    virtual float getRefraction() const {
        return refraction;
    }
    virtual float getShine() const {
        return shininess;
    }
    virtual Vector3f getSpecular() const {
        return specularColor;
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
        shaded = lightColor * diffuseColor * std::max(0.0f, Vector3f::dot(N, L)) + lightColor * specularColor * std::pow(std::max(0.0f, Vector3f::dot(R, V)), shininess);

        return shaded;
    }


protected:
    Vector3f diffuseColor;
    Type refl = DIFF;
    Vector3f specularColor = Vector3f::ZERO;
    float shininess = 0.0f;
    Vector3f emission = Vector3f::ZERO;

    float refraction = 1.0f; // 折射率
};




#endif // MATERIAL_H
