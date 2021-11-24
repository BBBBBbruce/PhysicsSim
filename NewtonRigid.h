
#ifndef NEWTONRIGID_H
#define NEWTONRIGID_H

#include"ExternalLib/pugixml/pugixml.hpp"

typedef pugi::xml_node xml;

#include "Engine.h"
class NewtonRigid //:
    //public Engine
{
private:
    vector<StaticObj>StaticVec;
    vector<DynamicObj>DynamicVec;
    xml node;

public:
    NewtonRigid();
    NewtonRigid(xml currentfig);
    void run(float time);// in seconds
    bool collision_detection();

    void ParseWorld(xml objectstree);
    xml ExportWorld();
};

#endif 

