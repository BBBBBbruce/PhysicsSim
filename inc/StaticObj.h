
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
    StaticObj(string n, string path, glm::vec3 pos, glm::vec3 sc, glm::vec3 rot);
    void initialise();
    void displayinfo();
    json tojson();
};
#endif

