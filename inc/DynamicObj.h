
#ifndef DYNAMICOBJ_H
#define DYNAMICOBJ_H

#include "Objects.h"
#include <tuple>

class DynamicObj :
    public Objects
{
private:
    Eigen::Vector3f linear_velocity;
    Eigen::Vector3f angular_velocity;
    Eigen::Vector3f mass_centre;
    float mass;

public:
    DynamicObj();
    //DynamicObj(string n, string path, float m);
    DynamicObj(string n,
        Eigen::MatrixXf pos,
        Eigen::MatrixXi tet,
        Eigen::MatrixXi tri,
        Eigen::MatrixXi tritag,
        Eigen::MatrixXi tettag,
        std::vector<std::string> xfields,
        std::vector<std::string> efields,
        std::vector<Eigen::MatrixXf> xf,
        std::vector<Eigen::MatrixXf> trif,
        std::vector<Eigen::MatrixXf> tetf,
        Eigen::Vector3f vel,
        Eigen::Vector3f cm,
        Eigen::Vector3f angular_vel,
        float m);
    Eigen::Vector3f get_linear_velocity();
    Eigen::MatrixXi get_tetrahedrons();
    Eigen::Vector3f get_angular_velocity();
    float get_mass();
    Eigen::Vector3f get_cm();
    void updatestate(Eigen::Vector3f pos, Eigen::Vector3f v, bool collide, float t, int seq);
    void update_state(Eigen::MatrixXf x, Eigen::Vector3f v,  Eigen::Vector3f w, Eigen::Vector3f cm);

    void writemsh(string p);
    void ToViewer(Eigen::MatrixXf& vertices, Eigen::MatrixXi& faces);
    tuple<Eigen::MatrixXf, Eigen::MatrixXi> Get_ViewMatrix();
    //void rotate(float t);
    
    
};

#endif
