
#ifndef COORDS_H
#define COORDS_H

#include<iostream>
#include<string>
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

