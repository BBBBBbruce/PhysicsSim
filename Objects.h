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
	coords position;
	coords velocity;
	float mass;

public:
	Objects(string n, string path, float m);
	Objects(string n, string path,coords pos,coords v, float m);
	void initialise();
	string get_name();
	string get_loadingpath();
	coords get_position();
	coords get_velocity();
	float get_mass();
};

#endif

