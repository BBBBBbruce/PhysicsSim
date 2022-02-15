
#ifndef DYNAMICOBJ_H
#define DYNAMICOBJ_H

#include "Objects.h"
#include <tuple>

class DynamicObj :
    public Objects
{
private:
    Eigen::Vector3d linear_velocity;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d mass_centre;
    double mass;
    //FEM
    Eigen::MatrixXd translation_velocity;

public:
    DynamicObj();
    DynamicObj(string n,
        Eigen::MatrixXd pos,
        Eigen::MatrixXi tet,
        Eigen::MatrixXi tri,
        Eigen::MatrixXi tritag,
        Eigen::MatrixXi tettag,
        std::vector<std::string> xfields,
        std::vector<std::string> efields,
        std::vector<Eigen::MatrixXd> xf,
        std::vector<Eigen::MatrixXd> trif,
        std::vector<Eigen::MatrixXd> tetf,
        Eigen::Vector3d vel,
        Eigen::Vector3d angular_vel,
        double m);
    Eigen::Vector3d get_linear_velocity();
    Eigen::MatrixXi get_tetrahedrons();
    Eigen::Vector3d get_angular_velocity();
    double get_mass();
    Eigen::Vector3d get_cm();
    void update_state(Eigen::MatrixXd x, Eigen::Vector3d v,  Eigen::Vector3d w);
    void writemsh(string p);
    void ToViewer(Eigen::MatrixXd& vertices, Eigen::MatrixXi& faces);
    tuple<Eigen::MatrixXd, Eigen::MatrixXi> Get_ViewMatrix();
    Eigen::Vector3d find_cm();
};

#endif
