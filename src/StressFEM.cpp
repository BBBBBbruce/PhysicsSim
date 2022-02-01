#include "StressFEM.h"


StressFEM::StressFEM()
{
}

StressFEM::StressFEM(string t_path)
{
    vector<StaticObj>StaticVec;
    vector<DynamicObj>DynamicVec;
    this->tpath = t_path;
}

void StressFEM::run(float time, int seq)
{
    // Implement the physics
}

bool StressFEM::collision_detection()
{
    // implement CD
    return false;
}

vector<StaticObj> StressFEM::getStaticObjs()
{
    return StaticVec;
}

vector<DynamicObj> StressFEM::getDynamicObjs()
{
    return DynamicVec;
}

void StressFEM::load_scene(int pre_seq)
{
}

void StressFEM::save_scene(int seq)
{
}

void StressFEM::InitConfigs(string targetpath, json currentconfig)
{

}
