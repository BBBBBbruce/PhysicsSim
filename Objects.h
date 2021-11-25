

#ifndef OBJECTS_H
#define OBJECTS_H

#include<string>
#include<stdexcept>
#include"Coords.h"

using namespace std;

class Objects
{
protected:
	string name;
	string loadingpath;
	glm::vec3 position;

public:
	Objects();
	virtual void initialise() = 0;
	string get_name();
	string get_loadingpath();
	glm::vec3 get_position();

};

#endif

