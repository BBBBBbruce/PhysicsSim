#include "NewtonRigid.h"

void NewtonRigid::run(float time)
{
}

bool NewtonRigid::collision_detection()
{
    return false;
}

pugi::xml_parse_result NewtonRigid::ExportWorld()
{

    return pugi::xml_parse_result();
}

void NewtonRigid::ParseWorld(pugi::xml_parse_result file)
{
    // parse all the objects and push them into vectors
}
