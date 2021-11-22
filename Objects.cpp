#include "Objects.h"


Objects::Objects()
{
	throw std::runtime_error(std::string("Failed: how do you know base class? ") );
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

