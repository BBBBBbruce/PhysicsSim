
#ifndef DYNAMICOBJ_H
#define DYNAMICOBJ_H

#include "Objects.h"
#include <tuple>

class DynamicObj :
    public Objects
{
private:
    Eigen::MatrixXf linear_velocity;
    Eigen::Vector3f angular_velocity;
    Eigen::Vector3f rotating_vector;
    Eigen::MatrixXf rotating_point;
    Eigen::Matrix3f gravity_centre;
    float mass;

public:
    DynamicObj();
    //DynamicObj(string n, string path, float m);
    
    DynamicObj(string n, Eigen::MatrixXf pos, Eigen::MatrixXi tet, Eigen::MatrixXi tri, Eigen::MatrixXi tritag, Eigen::MatrixXi tettag, std::vector<std::string> xfields, std::vector<std::string> efields, std::vector<Eigen::MatrixXf> xf, std::vector<Eigen::MatrixXf>trif, std::vector<Eigen::MatrixXf> tetf, Eigen::MatrixXf vel, float m);
    Eigen::MatrixXf get_linear_velocity();
    Eigen::MatrixXi get_tetrahedrons();
    float get_mass();
    void updatestate(Eigen::Vector3f pos, Eigen::Vector3f v, bool collide, float t, int seq);
    void simulate_linear(Eigen::Vector3f pos, Eigen::Vector3f v, bool collide,float t);
    void simulate_angular();
    void writemsh(string p, string v);
    void ToViewer(Eigen::MatrixXf& vertices, Eigen::MatrixXi& faces);
    tuple<Eigen::MatrixXf, Eigen::MatrixXi> Get_ViewMatrix();

    
    
};

#endif
