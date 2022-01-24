
#ifndef NEWTONRIGID_H
#define NEWTONRIGID_H

#include "Engine.h"

using json = nlohmann::json;
class NewtonRigid :
   public Engine
{
private:

    string tpath;
    //default
    float e; // restitution coefficient
    float tc; // collision time
    float u; // friction

public:
    NewtonRigid();
    NewtonRigid(string t_path);
    void run(float time, int seq);// in seconds
    void set_physics_params(float restitution, float collision_t, float friction);
    bool collision_detection();
    //void ParseWorld(json objectlist);
    vector<StaticObj> getStaticObjs();
    vector<DynamicObj> getDynamicObjs();
    void load_scene(int pre_seq);
    void save_scene(int seq);
    void create_aabb_tree();
    void reset();

};

#endif 

