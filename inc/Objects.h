

#ifndef OBJECTS_H
#define OBJECTS_H


#include<string>
#include"Coords.h"

using namespace std;

class Objects
{
protected:
	
	Eigen::MatrixXf position;
	Eigen::MatrixXi tetrahedral;
	//other unnecessary stuff, might be useful later:
	Eigen::MatrixXi Tri;
	Eigen::VectorXi TriTag, TetTag;
	std::vector<std::string> XFields, EFields;
	std::vector<Eigen::MatrixXf> XF, TriF, TetF;

public:
	string name;
	Objects();
	string get_name();
	Eigen::MatrixXf get_position();

};

Eigen::MatrixXf double2float(const Eigen::MatrixXd& matrix);
Eigen::MatrixXd float2double(const Eigen::MatrixXf& matrix);
vector<Eigen::MatrixXd> cast2double(const vector<Eigen::MatrixXf>& vec);
vector<Eigen::MatrixXf> cast2float (const vector<Eigen::MatrixXd>& vec);

#endif