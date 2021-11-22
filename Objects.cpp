#include "Objects.h"



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
	return postion;
}

coords Objects::get_velocity()
{
	return velocity;
}

float Objects::get_mass()
{
	return mass;
}
