#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>
#include <random>

#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"
#include "material.hpp"
#include "ray.hpp"

#include <string>
#include <vector>


# define samps 1000
# define M_PI 3.14159265358979323846

using namespace std;

double get_random(std::mt19937 gen = std::mt19937(std::random_device()()))
{
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    double random_number = distribution(gen);
    assert (random_number >= 0.0 && random_number <= 1.0);
    return random_number;
}

bool RR(double p){ return get_random() < p; }


Vector3f radiance(SceneParser* sceneParser, const Ray &camRay, int depth) {
    depth++;
    double t;
    Group* group = sceneParser -> getGroup();
    Hit hit;

    bool isIntersect = group->intersect(camRay, hit, 0);
    if(!isIntersect)
    {
        return sceneParser -> getBackgroundColor();
    }

    t = hit.getT();
    Vector3f point = camRay.pointAtParameter(t);
    Vector3f normal = hit.getNormal();
    normal = Vector3f::dot(camRay.getDirection(), normal) < 0 ? normal : normal * -1;

    Vector3f color = hit.getMaterial()->getDiffuseColor();
    float p = color.getMax();
    assert(p > 0 && p <= 1.0);

    if(depth > 5)
    {
        if(RR(p))
        {
            color = color * (1.0/p);
        }
        else
        {
            // 是光源则返回光源颜色 否则返回黑色
            return hit.getMaterial()->getEmission();
        }
    }

    // 漫反射
    if(hit.getMaterial()->getRefl() == DIFF)
    {
        double r1 = 2 * M_PI * get_random();
        double r2 = get_random();
        double r2s = sqrt(r2);
        Vector3f w = normal;
        float fabs_w_x = (float)w.x() > 0.0 ? w.x() : -w.x();
        Vector3f u = Vector3f::cross(( fabs_w_x > 0.1 ? Vector3f(0, 1, 0) : Vector3f(1, 0, 0)), w).normalized();
        Vector3f v = Vector3f::cross(w, u);
        Vector3f d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();
        return hit.getMaterial()->getEmission() + color * radiance(sceneParser, Ray(point, d), depth);
    }

    // 镜面反射
    else if(hit.getMaterial()->getRefl() == SPEC)
    {
        // 镜面反射方向
        Vector3f spec_d = camRay.getDirection() - normal * 2 * Vector3f::dot(normal, camRay.getDirection());
        return hit.getMaterial()->getEmission() + color * radiance(sceneParser, Ray(point, spec_d), depth);
    }

    // 折射
    else if (hit.getMaterial()->getRefl() == REFR)
    {
        Ray reflRay = Ray(point, camRay.getDirection() - normal * 2 * Vector3f::dot(normal, camRay.getDirection()));
        bool into = Vector3f::dot(normal, reflRay.getDirection()) > 0;
        float nc = 1;
        float nt = 1.5;
        float nnt = into ? nc / nt : nt / nc;
        float ddn = Vector3f::dot(camRay.getDirection(), normal);
        float cos2t = 1 - nnt * nnt * (1 - ddn * ddn);
        if(cos2t < 0)
        {
            return hit.getMaterial()->getEmission() + color * radiance(sceneParser, reflRay, depth);
        }

        Vector3f tdir = (camRay.getDirection() * nnt - normal * ((into ? 1 : -1) * (ddn * nnt + sqrt(cos2t)))).normalized();
        float a = nt - nc;
        float b = nt + nc;
        float R0 = a * a / (b * b);
        float c = 1 - (into ? -ddn : Vector3f::dot(tdir, normal));
        float Re = R0 + (1 - R0) * c * c * c * c * c;
        float Tr = 1 - Re;
        float P = 0.25 + 0.5 * Re;
        float RP = Re / P;
        float TP = Tr / (1 - P);

        if(depth > 2)
        {
            if(RR(P))
            {
                return hit.getMaterial()->getEmission() + color * radiance(sceneParser, reflRay, depth) * RP;
            }
            else
            {
                return hit.getMaterial()->getEmission() + color * radiance(sceneParser, Ray(point, tdir), depth) * TP;
            }
        }
        else
        {
            return hit.getMaterial()->getEmission() + color * (radiance(sceneParser, reflRay, depth) * Re + radiance(sceneParser, Ray(point, tdir), depth) * Tr);
        }
    }
    return Vector3f::ZERO;
}



int main(int argc, char *argv[]) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 3) {
        cout << "Usage: ./bin/PA1 <input scene file> <output bmp file>" << endl;
        return 1;
    }
    string inputFile = argv[1];
    string outputFile = argv[2];  // only bmp is allowed.

    // TODO: Main RayCasting Logic
    // First, parse the scene using SceneParser.
    // Then loop over each pixel in the image, shooting a ray
    // through that pixel and finding its intersection with
    // the scene.  Write the color at the intersection to that
    // pixel in your output image.

    cout << "start reading" << endl;
    SceneParser sceneParser(inputFile.c_str());
    Camera *camera = sceneParser.getCamera();
    Image *image = new Image(camera->getWidth(), camera->getHeight());

    cout << "start drawing" << endl;
    //  框架
    for(int x = 0; x < camera->getWidth(); ++x)
    {

        for(int y = 0; y < camera-> getHeight(); ++y)
        {
            Vector3f color = Vector3f::ZERO;

            vector<pair<double, double>> dots; // 采样点
            dots.push_back(pair<double, double>((x + 0.25) / camera -> getWidth() - 0.5, (y + 0.25) / camera -> getHeight() - 0.5));
            dots.push_back(pair<double, double>((x + 0.75) / camera -> getWidth() - 0.5, (y + 0.25) / camera -> getHeight() - 0.5));
            dots.push_back(pair<double, double>((x + 0.25) / camera -> getWidth() - 0.5, (y + 0.75) / camera -> getHeight() - 0.5));
            dots.push_back(pair<double, double>((x + 0.75) / camera -> getWidth() - 0.5, (y + 0.75) / camera -> getHeight() - 0.5));


            for(pair<double, double> item : dots)
            {
                // 采样
                for(int loop = 0; loop < samps; loop++)
                {
                    double r1 = 2 * get_random();
                    double dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
                    double r2 = 2 * get_random();
                    double dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);
                    dx = (item.first + 0.5 + dx) / 2.0;
                    dy = (item.second + 0.5 + dy) / 2.0;



                    Ray camRay = camera -> generateRay(Vector2f((x + dx), (y + dy)));

                    Vector3f r = radiance(&sceneParser, camRay, 0) * (1.0 / samps);
                    color += r;
                }
            }
            color = color / 4.0;
            image->SetPixel(x, y, color);
        }
    }

    image->SaveImage(outputFile.c_str());


    cout << "Hello! Computer Graphics!" << endl;
    return 0;
}

