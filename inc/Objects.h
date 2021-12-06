

#ifndef OBJECTS_H
#define OBJECTS_H
#define GLM_ENABLE_EXPERIMENTAL

#include "glm/gtx/string_cast.hpp"
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
	glm::vec3 scale;
	glm::vec3 rotation;

public:
	Objects();
	virtual void initialise() = 0;
	string get_name();
	string get_loadingpath();
	glm::vec3 get_position();

};

#endif

