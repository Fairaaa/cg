#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include <vecmath.h>
#include <float.h>
#include <cmath>


class Camera {
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH) {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up).normalized();
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point) = 0;
    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }

protected:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
};

// TODO: Implement Perspective camera finish?
// You can add new functions or variables whenever needed.
class PerspectiveCamera : public Camera {

public:
    PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
            const Vector3f &up, int imgW, int imgH, float angle) : Camera(center, direction, up, imgW, imgH) {
        // angle is in radian.
        this->angle = angle;

    }
    // 给定图像坐标，返回到该点的光线
    Ray generateRay(const Vector2f &point) override {
        // 相机空间下的射线
        float fx = this->height /( 2* tan(angle / 2));
        float fy = this->height /( 2 * tan(angle / 2));
        Vector3f dir((point.x() - this->width/2) / fx, ( this->height / 2 - point.y()) / fy, 1);
        Vector3f dirr = dir.normalized();
        
        // 转换到世界空间下的射线
        Vector3f worldDir = dirr.x() * this->horizontal - dirr.y() * this->up + dirr.z() * this->direction;
        return Ray(this->center, worldDir);


    }

protected:
    float angle;
};

#endif //CAMERA_H
