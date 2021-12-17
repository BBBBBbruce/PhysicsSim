
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
    Tree aabb_tree;
    

public:
    NewtonRigid();
    NewtonRigid(string t_path);
    void run(float time, int seq);// in seconds
    bool collision_detection();
    //void ParseWorld(json objectlist);
    vector<StaticObj> getStaticObjs();
    vector<DynamicObj> getDynamicObjs();
    void ShowObjectsInfo();
    void load_scene(int pre_seq);
    void save_scene(int seq);
    void create_aabb_tree();
    void reset();

};

#endif 

