
#ifndef COORDS_H
#define COORDS_H

#include<iostream>
#include <direct.h>

#include<glm/vec3.hpp>
#include"json.hpp"
#include "visualisation.h"
using namespace std;
using json = nlohmann::json;

struct coords
{
	float x;
	float y;
	float z;

	coords& operator =(const coords& a)
	{
		x = a.x;
		y = a.y;
		z = a.z;
		return *this;
	}

};

#endif


