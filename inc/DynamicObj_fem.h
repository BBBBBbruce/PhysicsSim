#ifndef DYNAMICOBJ_FEM_H
#define DYNAMICOBJ_FEM_H

#pragma once
#include "DynamicObj.h"

using namespace std;
using namespace Eigen;


class DynamicObj_fem :
    public DynamicObj
{
protected:

    Eigen::MatrixXd deform_velocity;
    Eigen::MatrixXd RigidPos;
    VectorXd mass;
    double Young;
    double Poisson;
    MatrixXd YModulus;

public:
    DynamicObj_fem();
    DynamicObj_fem(string n,
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
        Eigen::Vector3d linear_vel,
        Eigen::Vector3d angular_vel,
        Eigen::MatrixXd Deform_vel,
        Eigen::MatrixXd rigid_pos,
        VectorXd m,
        double y,
        double po,
        MatrixXd ym
    );

    Eigen::Vector3d get_linear_velocity();
    Eigen::MatrixXi get_tetrahedrons();
    Eigen::Vector3d get_angular_velocity();
    Eigen::MatrixXd get_deformation_velocity();
    Eigen::MatrixXd get_rigid_position();
    double get_youngs_modulus();
    VectorXd get_mass();
    double get_poisson_ratio();
    MatrixXd get_YModulus();
    Eigen::Vector3d get_cm();

    //yet be update
    void update_state(Eigen::MatrixXd x, Eigen::Vector3d v, Eigen::Vector3d w);
    void save_current_state(string p);
    void save_debug_info();
};


#endif

