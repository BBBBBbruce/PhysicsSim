#pragma once
#ifndef COORDS_H
#define COORDS_H

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

