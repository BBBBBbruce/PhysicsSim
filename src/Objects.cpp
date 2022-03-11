#include "Objects.h"

Objects::Objects()
{
	//throw std::runtime_error(std::string("Failed: how do you know base class? ") );
}



string Objects::get_name()
{
	return name;
}



Eigen::MatrixXd Objects::get_position()
{
	return position;
}



