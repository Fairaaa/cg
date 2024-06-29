#ifndef GROUP_H
#define GROUP_H


#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>


// TODO: Implement Group - add data structure to store a list of Object* 
// 所有可以与光线相交的物体放在这里
class Group : public Object3D {

public:

    bool hasLight = false;
    Group() {
    }

    explicit Group (int num_objects) {
        objects.resize(num_objects);
    }

    ~Group() override {
        for (auto obj : objects) {
            delete obj;
        }
    }

    // 和所有objects求一遍交点
    bool intersect(const Ray &r, Hit &h, float tmin) override {
        bool flag = false;
        for (auto obj : objects) {
            if (obj->intersect(r, h, tmin)) {
                flag = true;
            }
        }
        return flag;
    }

    void addObject(int index, Object3D *obj) {
        objects[index] = obj;
    }

    int getGroupSize() {
        return objects.size();
    }

private:
    std::vector<Object3D*> objects;
};

#endif
	
