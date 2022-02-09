#pragma once
#ifndef COLLISION_H
#define COLLISION_H

#include "visualisation.h"
using namespace std;
using namespace Eigen;



tuple<bool, Vector3f> CD_bowl(MatrixXf vertices);

tuple<bool, Vector3f> CD_table(MatrixXf vertices);

tuple<bool, Vector3f> CD_bowl_wide(MatrixXf vertices);

tuple<bool, Vector3f> CD_vshape(MatrixXf vertices);

tuple<bool, vector<int>> CD_table_FEM(MatrixXf vertices);

#endif
