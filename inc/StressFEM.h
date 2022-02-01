#ifndef STRESSFEM_H
#define STRESSFEM_H
#pragma once


#include "Engine.h"


class StressFEM :
    public Engine
{

public:
    StressFEM();
    StressFEM(string t_path);
    void run(float time, int seq);// in seconds
    bool collision_detection();
    vector<StaticObj> getStaticObjs();
    vector<DynamicObj> getDynamicObjs();
    void load_scene(int pre_seq);
    void save_scene(int seq);
    void InitConfigs(string targetpath, json currentconfig);

};

#endif // !STRESSFEM