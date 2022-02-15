#pragma once
#ifndef COLLISION_H
#define COLLISION_H

#include "visualisation.h"
using namespace std;
using namespace Eigen;



tuple<bool, Vector3d> CD_bowl(MatrixXd vertices);

tuple<bool, Vector3d> CD_table(MatrixXd vertices);

tuple<bool, Vector3d> CD_bowl_wide(MatrixXd vertices);

tuple<bool, Vector3d> CD_vshape(MatrixXd vertices);

tuple<bool, vector<int>> CD_table_FEM(MatrixXd vertices);

#endif
