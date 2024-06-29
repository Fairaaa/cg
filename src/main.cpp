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


// # define samps 1000
# define M_PI 3.14159265358979323846
# define epi 1e-5
# define samps_light 20

using namespace std;

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<double> distribution(0.0, 1.0);

int unsamps_cnt = 0;

double get_random()
{
    double random_number = distribution(gen);
    assert(random_number >= 0.0 && random_number <= 1.0);
    return random_number;
}
bool RR(double p){ return get_random() < p; }

Vector3f Emission(Vector3f point)
{
    if(point.y() > 2 - epi && point.y() < 2 + epi && point.x() * point.x() + point.z() * point.z() < 0.25 + epi)
    {
        return Vector3f(12, 12, 12);
    }
    return Vector3f::ZERO;
}


Vector3f radiance(SceneParser* sceneParser, const Ray &camRay, int depth, bool ifNee = false) {
    depth++;
    double t;
    Group* group = sceneParser -> getGroup();
    Hit hit;

    bool isIntersect = group->intersect(camRay, hit, epi);
    if(!isIntersect) return sceneParser -> getBackgroundColor();

    t = hit.getT();
    Vector3f point = camRay.pointAtParameter(t);
    Vector3f normal = hit.getNormal();
    normal = Vector3f::dot(camRay.getDirection(), normal) < 0 ? normal : normal * -1;

    Vector3f color = hit.getMaterial()->getDiffuseColor();
    double p = color.getMax();
    assert(p > 0 && p <= 1.0);

    // 递归终止于漫反射：找到光源或者递归深度大于5且概率小于p
    if(depth > 5)
    {
        if(RR(p))
        {
            color = color * (1.0/p);
            if(color.getMax() < epi || depth > 200)
            {
                return Emission(point);
            }
        }
        // 是光源则返回光源颜色 否则返回黑色
        else return Emission(point);
    }

    // 漫反射
    if(hit.getMaterial()->getRefl() == DIFF)
    {
        Vector3f indirectcolor = Vector3f::ZERO;

        // 随机采样
        double r1 = 2 * M_PI * get_random();
        double r2 = get_random();
        double r2s = sqrt(r2);
        Vector3f w = normal;
        double fabs_w_x = (double)w.x() > 0.0 ? w.x() : -w.x();
        Vector3f u = Vector3f::cross(( fabs_w_x > 0.1 ? Vector3f(0, 0, 1) : Vector3f(1, 0, 0)), w).normalized();
        Vector3f v = Vector3f::cross(w, u);
        Vector3f d = (u * cos(r1) * r2s + v * sin(r1) * r2s + w * sqrt(1 - r2)).normalized();


        // 考虑NEE且当前交点为光源
        if(depth == 2 && ifNee && Emission(point).getMax() > epi)
        {
            unsamps_cnt++;
            return Vector3f::ZERO;
        }
        else
        {   
            indirectcolor += radiance(sceneParser, Ray(point, d), depth);
        }
        // 间接光照
        return Emission(point) + color * indirectcolor;
    }

    // 镜面反射
    else if(hit.getMaterial()->getRefl() == SPEC)
    {
        // 镜面反射方向
        Vector3f spec_d = camRay.getDirection() - normal * 2 * Vector3f::dot(normal, camRay.getDirection());
        return Emission(point) + color * radiance(sceneParser, Ray(point, spec_d), depth);
    }

    // 折射
    else if (hit.getMaterial()->getRefl() == REFR)
    {
        Ray reflRay = Ray(point, camRay.getDirection() - normal * 2 * Vector3f::dot(normal, camRay.getDirection()));
        bool into = Vector3f::dot(normal, reflRay.getDirection()) > 0;
        float nc = 1;
        float nt = hit.getMaterial()->getRefraction();
        float nnt = into ? nc / nt : nt / nc;
        float ddn = Vector3f::dot(camRay.getDirection(), normal);
        float cos2t = 1 - nnt * nnt * (1 - ddn * ddn);
        if(cos2t < 0)
        {
            return Emission(point) + color * radiance(sceneParser, reflRay, depth);
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
            if(get_random() < P){
                return Emission(point) + color * radiance(sceneParser, reflRay, depth) * RP;
            }
            else{
                return Emission(point) + color * radiance(sceneParser, Ray(point, tdir), depth) * TP;
            }
        }
        else
        {
            return Emission(point) + color * (radiance(sceneParser, reflRay, depth) * Re + radiance(sceneParser, Ray(point, tdir), depth) * Tr);
        }
    }
    return Vector3f::ZERO;
}



int main(int argc, char *argv[]) {
    // 是否使用NEE采样
    bool ifUseNee = true;

    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 4) {
        cout << "Usage: ./bin/PA1 <input scene file> <output bmp file>" << endl;
        return 1;
    }
    string inputFile = argv[1];
    string outputFile = argv[2];  // only bmp is allowed.
    int samps = atoi(argv[3]);

    cout << "start reading" << endl;
    SceneParser sceneParser(inputFile.c_str());
    Camera *camera = sceneParser.getCamera();
    Image *image = new Image(camera->getWidth(), camera->getHeight());

    cout << "start drawing" << endl;
    float startT = clock();
    for(int x = 0; x < camera->getWidth(); ++x)
    {   
        #pragma omp parallel for schedule(dynamic) 
        for(int y = 0; y < camera-> getHeight(); ++y)
        {
            
            // unsigned short Xi[3] = {0, 0, static_cast<unsigned short>(y * y * y)};

            Vector3f color = Vector3f::ZERO;
            Vector3f directcolor = Vector3f::ZERO;
            Vector3f indirectcolor = Vector3f::ZERO;

            // 是否进行了NEE采样
            bool ifNee = false;

            // 对光源直接采样
            // 采样条件：漫反射材质且接受到光源直接照射
            Hit hitpoint; // 相机视线和物体交点
            Ray camDRay = camera -> generateRay(Vector2f(x, y));
            bool intersect = sceneParser.getGroup()->intersect(camDRay, hitpoint, epi);
            bool flag = intersect ? (hitpoint.getMaterial()->getRefl() == DIFF) : false;
            // 采样点为漫反射材质
            if(ifUseNee && flag)
            {
                Vector3f current_point = camDRay.pointAtParameter(hitpoint.getT());
                int samps_light_cnt = 0;
                // 光源为(0,2,0)为圆心，0.5为半径的圆盘 从光源上随机取样
                for(int i = 0; i < samps_light; i++)
                {
                    double r1 = 2 * M_PI * get_random();
                    double r2 = get_random();
                    double r2s = sqrt(r2);
                    // 采样光源点
                    Vector3f lightpoint = Vector3f(0, 2, 0) + Vector3f(0.5 * cos(r1) * r2s, 0, 0.5 * sin(r1) * r2s);
                    // 判断光线是否被遮挡
                    assert(lightpoint.y() == 2 && lightpoint.x() * lightpoint.x() + lightpoint.z() * lightpoint.z() < 0.25);
                    Vector3f shadowdir = (lightpoint - current_point).normalized();
                    Ray shadowRay = Ray(current_point, shadowdir);

                    Hit shadowhit;
                    assert(sceneParser.getGroup()->intersect(shadowRay, shadowhit, epi) == true);

                   Vector3f shadowHitPoint = shadowRay.pointAtParameter(shadowhit.getT());

                    // 从相交点向光源发射光线 如果最近交点来自天花板则排除
                    bool isShadow = shadowHitPoint.y() < 2.0 - epi;
                    // 未被遮挡
                
                    if(!isShadow)
                    {
                        // 交点到光源的距离
                        float distance = (shadowHitPoint - current_point).length();
                        samps_light_cnt++;
                        // 进行了NEE采样
                        ifNee = true;
                        double cos_theta = Vector3f::dot(hitpoint.getNormal(), shadowdir);
                        // 光源直接照射
                        float pdf = 1.0 / (M_PI * 0.25 * distance * distance);
                        if(cos_theta > 0)
                        {
                            directcolor += hitpoint.getMaterial()->getDiffuseColor() * cos_theta * pdf ;
                        }
                    }
                }
                if(samps_light_cnt > 0){
                    directcolor = directcolor / samps_light_cnt;
                }
            }

            vector<pair<double, double>> dots; // 采样点
            dots.push_back(pair<double, double>((x + 0.25) / camera -> getWidth() - 0.5, (y + 0.25) / camera -> getHeight() - 0.5));
            dots.push_back(pair<double, double>((x + 0.75) / camera -> getWidth() - 0.5, (y + 0.25) / camera -> getHeight() - 0.5));
            dots.push_back(pair<double, double>((x + 0.25) / camera -> getWidth() - 0.5, (y + 0.75) / camera -> getHeight() - 0.5));
            dots.push_back(pair<double, double>((x + 0.75) / camera -> getWidth() - 0.5, (y + 0.75) / camera -> getHeight() - 0.5));

            // 间接光照采样
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
                    
                    Vector3f r = radiance(&sceneParser, camRay, 0,ifNee) * (1.0 / (samps - unsamps_cnt));
                    indirectcolor += r.clamp();
                }
            }
            indirectcolor = indirectcolor / 4.0;

            color = directcolor + indirectcolor;
            image->SetPixel(x, y, color);
        }
    }

    image->SaveImage(outputFile.c_str());

    float endT = clock();


    cout << "Hello! Computer Graphics!" << endl;
    cout << "Time cost: " << (endT - startT) / CLOCKS_PER_SEC << "s" << endl;
    // cout << "采样光源成功率" << samps_light_cnt / samps_light << endl;
    // cout << "间接光照采样成功率" << (samps - unsamps_cnt) / samps << endl;
    return 0;
}
