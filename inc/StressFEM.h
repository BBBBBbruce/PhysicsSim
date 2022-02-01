#ifndef STRESSFEM_H
#define STRESSFEM_H
#pragma once


#include "Engine.h"
#include "Collision.h"

class StressFEM :
    public Engine
{

private:
    vector<Eigen::MatrixXf>TransVelVec;

public:
    StressFEM();
    StressFEM(string t_path);
    void run(float delta_t, int seq);// in seconds
    vector<StaticObj> getStaticObjs();
    vector<DynamicObj> getDynamicObjs();
    void load_scene(int pre_seq);
    void save_scene(int seq);
    void InitConfigs(string targetpath, json currentconfig);

};

#endif // !STRESSFEM