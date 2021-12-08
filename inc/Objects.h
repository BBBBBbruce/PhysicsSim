

#ifndef OBJECTS_H
#define OBJECTS_H


#include<string>
#include"Coords.h"

using namespace std;

class Objects
{
protected:
	string name;
	string loadingpath;
	Eigen::Vector3f position;
	Eigen::Vector3f scale;
	Eigen::Vector3f rotation;

public:
	Objects();
	string get_name();
	string get_loadingpath();
	Eigen::Vector3f get_position();

};

#endif