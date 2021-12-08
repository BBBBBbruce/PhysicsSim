
#ifndef DYNAMICOBJ_H
#define DYNAMICOBJ_H

#include "Objects.h"


class DynamicObj :
    public Objects
{
private:
    Eigen::Vector3f velocity;
    float mass;

public:
    DynamicObj();
    DynamicObj(string n, string path, float m);
    DynamicObj(string n, string path, Eigen::Vector3f pos, Eigen::Vector3f sc, Eigen::Vector3f rot, Eigen::Vector3f vel, float m);
    Eigen::Vector3f get_velocity();
	float get_mass();
    void updatestate(Eigen::Vector3f pos, Eigen::Vector3f v);
    void displayinfo();
    json tojson();
    
    
};

#endif
