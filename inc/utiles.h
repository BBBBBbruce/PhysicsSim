#ifndef UTILES_H
#define UTILES_H
#pragma once

#include <Eigen/Dense>
#include <direct.h>
#include <algorithm>
#include <random>
#include <map>
#include <fstream>
#include <ostream>
#include <iostream>
using namespace std;
using namespace Eigen;

Eigen::MatrixXf double2float(const Eigen::MatrixXd& matrix);
Eigen::MatrixXd float2double(const Eigen::MatrixXf& matrix);
vector<Eigen::MatrixXd> cast2double(const vector<Eigen::MatrixXf>& vec);
vector<Eigen::MatrixXf> cast2float(const vector<Eigen::MatrixXd>& vec);
void make_dir_win(string targetpath, int seq);
inline Eigen::Vector3d gravity = { 0.0,-10.0,0.0 };
Eigen::Matrix4d rotate_eigen_api(Vector3d theta);
void rotate(MatrixXd& x, Vector3d& theta, Vector3d& cm);
Eigen::Vector3d rowwise_projection(Eigen::Vector3d vec, Eigen::Vector3d dir_vec);
Eigen::MatrixXd projection(Eigen::MatrixXd matrix, Eigen::Vector3d dir_vec);
Eigen::Vector3d project_cos(Eigen::Vector3d vec, Eigen::Vector3d dir_vec);
Eigen::Vector3d project_sin(Eigen::Vector3d vec, Eigen::Vector3d dir_vec) ;
float TetrahedronElementVolume(MatrixXd tet_vertices);
MatrixXi tet_re_order(MatrixXi tet, MatrixXd p1, MatrixXd p2, MatrixXd p3, MatrixXd p4);
MatrixXd TetrahedronElementStiffness(MatrixXd YModulus, MatrixXd vertices);
void TetrahedronAssemble(MatrixXd& K, MatrixXd k, MatrixXi indices);
MatrixXd TetrahedronElementStresses(MatrixXd YModulus, MatrixXd vertices, MatrixXd u);
Matrix<double, 1, 3> TetrahedronElementPStresses(Matrix<double, 6, 1> sigma);
void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove);
void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove);
MatrixXd insertRow(MatrixXd  matrix, int row, MatrixXd r);
void Xd2csv(string fileName, MatrixXd  matrix);
void Xf2csv(string fileName, MatrixXd  matrix);
MatrixXd XfFromcsv(string fileToOpen);
MatrixXd HookeLaw(MatrixXd epsilon, Matrix<double, 6, 6 > modulus);
MatrixXd apply_poisson(MatrixXd d, double p);
bool point_in_triangle(Matrix<double, 1, 3> p, Matrix<double, 1, 3> V0, Matrix<double, 1, 3> V1, Matrix<double, 1, 3> V2);
tuple<bool, double> LinePlaneIntersection(
    Matrix<double, 1, 3> p,
    Matrix<double, 1, 3> V,
    Matrix<double, 1, 3> V1,
    Matrix<double, 1, 3> V2,
    Matrix<double, 1, 3> V3
);
tuple<bool, double, int> self_collision_tet(Matrix<double, 4, 3> X, Matrix<double, 4, 3> P);
MatrixXd mat2vec(MatrixXd mat, int row, int col);
MatrixXd vec2mat(MatrixXd mat, int row, int col);

#endif