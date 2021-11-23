
#ifndef NEWTONRIGID_H
#define NEWTONRIGID_H


#include "Engine.h"
class NewtonRigid :
    public Engine
{
    void run(float time);
    bool collision_detection();

};

#endif 

