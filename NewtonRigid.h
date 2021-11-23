
#ifndef NEWTONRIGID_H
#define NEWTONRIGID_H


#include "Engine.h"
class NewtonRigid :
    public Engine
{

public:
    void run(float time);// in seconds
    bool collision_detection();

    void ParseWorld(pugi::xml_parse_result file);
    pugi::xml_parse_result ExportWorld();
};

#endif 

