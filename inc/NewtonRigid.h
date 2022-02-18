
#ifndef NEWTONRIGID_H
#define NEWTONRIGID_H

#include "Engine.h"

using json = nlohmann::json;
class NewtonRigid :
   public Engine
{

public:
    NewtonRigid();
    NewtonRigid(string t_path);
    void run(float time, int seq);// in seconds
    bool collision_detection();
    //void ParseWorld(json objectlist);
    vector<StaticObj> getStaticObjs();
    vector<DynamicObj> getDynamicObjs();
    void load_scene(int pre_seq);
    void save_scene(int seq);
    void create_aabb_tree();
    void InitConfigs(string targetpath, json currentconfig, float tc);

};

#endif 

