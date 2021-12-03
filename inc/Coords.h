
#ifndef COORDS_H
#define COORDS_H

#include<iostream>
#include<string>
#include<glm/vec3.hpp>
#include"json.hpp"
using namespace std;

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

