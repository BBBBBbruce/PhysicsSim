
#ifndef STATICOBJ_H
#define STATICOBJ_H

#include "Objects.h"
using json = nlohmann::json;

class StaticObj :
    public Objects
{
public:
    StaticObj();
    StaticObj(string n, string path);
    StaticObj(string n, string path, Eigen::Vector3f pos, Eigen::Vector3f sc, Eigen::Vector3f rot);
    void displayinfo();
    json tojson();
};
#endif

