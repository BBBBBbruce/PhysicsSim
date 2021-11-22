#include "Objects.h"

Objects::Objects(string n, string path, float m)
{
	name = n;
	loadingpath = path;
	position = coords{ 0.,0.,0. };
	velocity = coords{ 0.,0.,0. };
	mass = m;
}

Objects::Objects(string n, string path, coords pos, coords v, float m)
{
	name = n;
	loadingpath = path;
	position = pos;
	velocity = v;
	mass = m;
}

void Objects::initialise()
{
	//: load model
}

string Objects::get_name()
{
	return name;
}

string Objects::get_loadingpath()
{
	return loadingpath;
}

coords Objects::get_position()
{
	return position;
}

coords Objects::get_velocity()
{
	return velocity;
}

float Objects::get_mass()
{
	return mass;
}
