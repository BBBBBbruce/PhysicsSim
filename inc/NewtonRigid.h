
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
    vector<StaticObj*> getStaticObjs();
    vector<DynamicObj*> getDynamicObjs();
    void InitConfigs(string targetpath, json currentconfig, float tc);
    void load_scene(int pre_seq);
    void save_scene(int seq);
    void run(float time, int seq);// in seconds

};

#endif 

