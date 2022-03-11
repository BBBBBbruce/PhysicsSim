

#ifndef OBJECTS_H
#define OBJECTS_H


#include<string>
#include"Coords.h"

using namespace std;

class Objects
{
protected:
	
	Eigen::MatrixXd position;
	Eigen::MatrixXi tetrahedrons;
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




#endif