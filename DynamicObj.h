#pragma once
#ifndef DYNAMICOBJ_H
#define DYNAMICOBJ_H

#include "Objects.h"
class DynamicObj :
    public Objects
{
private:
    coords velocity;
    float mass;

public:
    
    DynamicObj(string n, string path, float m);
    DynamicObj(string n, string path, coords pos, coords v, float m);
    coords get_velocity();
	float get_mass();
    void initialise();
};

#endif
