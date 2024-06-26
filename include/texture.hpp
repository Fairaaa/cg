#ifndef TEXTURE_H
#define TEXTURE_H

#include "vecmath.h"
using namespace std;

class texture {
    public:
        virtual ~texture() = default;
        virtual Vector3f value(float u, float v, const Vector3f &p) const = 0;
};

class solid_color : public texture {
    public: solid_color(const Vector3f &c) : color(c) {}
    solid_color(float red, float green, float blue) : solid_color(Vector3f(red, green, blue)) {}
    virtual Vector3f value(float u, float v, const Vector3f &p) const override {
        return color;
    }
    private:
        Vector3f color;
}

class checker_texture : public texture {
  public:
    checker_texture(double scale, shared_ptr<texture> even, shared_ptr<texture> odd)
      : inv_scale(1.0 / scale), even(even), odd(odd) {}

    checker_texture(double scale, const color& c1, const color& c2)
      : inv_scale(1.0 / scale),
        even(make_shared<solid_color>(c1)),
        odd(make_shared<solid_color>(c2))
    {}

    color value(double u, double v, const point3& p) const override {
        auto xInteger = int(std::floor(inv_scale * p.x()));
        auto yInteger = int(std::floor(inv_scale * p.y()));
        auto zInteger = int(std::floor(inv_scale * p.z()));

        bool isEven = (xInteger + yInteger + zInteger) % 2 == 0;

        return isEven ? even->value(u, v, p) : odd->value(u, v, p);
    }

  private:
    double inv_scale;
    shared_ptr<texture> even;
    shared_ptr<texture> odd;
};

#endif // TEXTURE_H