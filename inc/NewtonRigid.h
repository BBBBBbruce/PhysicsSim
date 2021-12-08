
#ifndef NEWTONRIGID_H
#define NEWTONRIGID_H

#include "Engine.h"

using json = nlohmann::json;
class NewtonRigid :
   public Engine
{
private:
    //vector<StaticObj>StaticVec;
    //vector<DynamicObj>DynamicVec;
    string tpath;
    

public:
    NewtonRigid();
    NewtonRigid(string t_path);
    void run(float time);// in seconds
    bool collision_detection();
    //void ParseWorld(json objectlist);
    vector<StaticObj> getStaticObjs();
    vector<DynamicObj> getDynamicObjs();
    void ShowObjectsInfo();
    void load_scene(time_t t_stamp);
    void save_scene(time_t t_stamp);
};

#endif 

