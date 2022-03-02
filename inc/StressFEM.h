#ifndef STRESSFEM_H
#define STRESSFEM_H
#pragma once


#include "Engine.h"
#include "Collision.h"

class StressFEM :
    public Engine
{

private:
    vector<Eigen::MatrixXd>currentVec;
    vector<Eigen::MatrixXd>lastVec;
    vector<Eigen::MatrixXd>RigidPosVec;
    vector<double> DampVec;
    VectorXd mass;
    double Young;
    double Poisson;
    MatrixXd YModulus;
    double zeta;
    //float damp;

public:
    StressFEM();
    StressFEM(string t_path);
    void run(float delta_t, int seq);// in seconds
    vector<StaticObj> getStaticObjs();
    vector<DynamicObj> getDynamicObjs();
    void load_scene(int pre_seq);
    void save_scene(int seq);
    void InitConfigs(string targetpath, json currentconfig, float tc);
    void reset();
    void set_physics_params(double d, double restitution, double collision_t, double friction, double young, double poisson);

};//

#endif // !STRESSFEM