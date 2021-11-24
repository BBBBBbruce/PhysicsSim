
#ifndef DYNAMICOBJ_H
#define DYNAMICOBJ_H

#include "Objects.h"
using json = nlohmann::json;

class DynamicObj :
    public Objects
{
private:
    coords velocity;
    float mass;

public:
    DynamicObj();
    DynamicObj(string n, string path, float m);
    DynamicObj(string n, string path, coords pos, coords v, float m);
    coords get_velocity();
	float get_mass();
    void initialise();
    void updatestate(coords pos, coords v);
    void displayinfo();
    json tojson();
    
};

#endif
