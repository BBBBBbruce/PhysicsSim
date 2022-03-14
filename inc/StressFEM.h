#ifndef STRESSFEM_H
#define STRESSFEM_H
#pragma once


#include "Engine.h"
#include "Collision.h"
#include "DynamicObj_fem.h"

class StressFEM :
    public Engine
{

private:
    std::vector<DynamicObj_fem*> DynamicVec;

public:
    StressFEM();
    StressFEM(string t_path);
    void run(float delta_t, int seq);// in seconds
    vector<StaticObj*> getStaticObjs();
    vector<DynamicObj_fem*> getDynamicObjs();
    void load_scene(int pre_seq);
    void save_scene(int seq);
    void InitConfigs(string targetpath, json currentconfig, float tc);
    void reset();
    void set_physics_params(double restitution, double collision_t, double friction);
    void rigid(float delta_t, int seq);
    void fem(float delta_t, int seq);
};//

#endif // !STRESSFEM