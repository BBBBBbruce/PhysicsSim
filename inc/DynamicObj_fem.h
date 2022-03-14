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

    MatrixXd deform_velocity;
    MatrixXd RigidPos;
    MatrixXd mass_vec;
    double mass_sum;
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
        MatrixXd m_vec,
        double m_sum,
        MatrixXd ym
    );

    Eigen::Vector3d get_linear_velocity();
    Eigen::MatrixXi get_tetrahedrons();
    Eigen::Vector3d get_angular_velocity();
    Eigen::MatrixXd get_deformation_velocity();
    Eigen::MatrixXd get_rigid_position();
    double get_youngs_modulus();
    MatrixXd get_mass_vec();
    double get_mass_sum();
    double get_poisson_ratio();
    MatrixXd get_YModulus();
    Eigen::Vector3d get_cm();

    //yet be update
    void update_state(Eigen::MatrixXd x, Eigen::Vector3d v, Eigen::Vector3d w);
    void save_current_state(string p);
    void save_debug_info();
};


#endif

