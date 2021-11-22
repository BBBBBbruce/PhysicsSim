#pragma once

#ifndef OBJECTS_H
#define OBJECTS_H

#include<string>
#include"Coords.h"

using namespace std;

class Objects
{
private:
	string name;
	string loadingpath;
	coords postion;
	coords velocity;
	float mass;

public:
	string get_name();
	string get_loadingpath();
	coords get_position();
	coords get_velocity();
	float get_mass();
};

#endif

