

#ifndef OBJECTS_H
#define OBJECTS_H


#include<string>
#include"Coords.h"

using namespace std;

class Objects
{
protected:
	
	Eigen::MatrixXd position;
	Eigen::MatrixXi tetrahedral;
	//other unnecessary stuff, might be useful later:
	Eigen::MatrixXi Tri;
	Eigen::VectorXi TriTag, TetTag;
	std::vector<std::string> XFields, EFields;
	std::vector<Eigen::MatrixXd> XF, TriF, TetF;

public:
	string name;
	Objects();
	string get_name();
	Eigen::MatrixXd get_position();

};

Eigen::MatrixXf double2float(const Eigen::MatrixXd& matrix);
Eigen::MatrixXd float2double(const Eigen::MatrixXf& matrix);
vector<Eigen::MatrixXd> cast2double(const vector<Eigen::MatrixXf>& vec);
vector<Eigen::MatrixXf> cast2float (const vector<Eigen::MatrixXd>& vec);
void make_dir_win(string targetpath, int seq);

inline Eigen::Vector3d gravity = { 0.0,-10.0,0.0 };

#endif