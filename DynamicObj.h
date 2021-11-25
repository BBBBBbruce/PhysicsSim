
#ifndef DYNAMICOBJ_H
#define DYNAMICOBJ_H

#include "Objects.h"
using json = nlohmann::json;

class DynamicObj :
    public Objects
{
private:
    glm::vec3 velocity;
    float mass;

public:
    DynamicObj();
    DynamicObj(string n, string path, float m);
    DynamicObj(string n, string path, glm::vec3 pos, glm::vec3 v, float m);
    glm::vec3 get_velocity();
	float get_mass();
    void initialise();
    void updatestate(glm::vec3 pos, glm::vec3 v);
    void displayinfo();
    json tojson();
    
    
};

#endif
