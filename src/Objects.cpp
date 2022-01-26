#include "Objects.h"


Objects::Objects()
{
	//throw std::runtime_error(std::string("Failed: how do you know base class? ") );
}



string Objects::get_name()
{
	return name;
}



Eigen::MatrixXf Objects::get_position()
{
	return position;
}

Eigen::MatrixXf double2float(const Eigen::MatrixXd& matrix)
{	
	Eigen::MatrixXf f = matrix.cast <float>();
	return f;
}

Eigen::MatrixXd float2double(const Eigen::MatrixXf& matrix)
{
	Eigen::MatrixXd d = matrix.cast <double>();
	return d;
}

vector<Eigen::MatrixXd> cast2double(const vector<Eigen::MatrixXf>& vec)
{	
	/*
	std::vector<NewColorSpacePoint> new_points;
	new_points.reserve(points.size());
	std::transform(points.begin(), points.end(),
	std::back_inserter(new_points), to_new_color);
	*/
	std::vector<Eigen::MatrixXd> dvec;
	dvec.reserve(vec.size());
	std::transform(vec.begin(), vec.end(),std::back_inserter(dvec), float2double);
	return dvec;
}

vector<Eigen::MatrixXf> cast2float(const vector<Eigen::MatrixXd>& vec)
{
	std::vector<Eigen::MatrixXf> fvec;
	fvec.reserve(vec.size());
	std::transform(vec.begin(), vec.end(), std::back_inserter(fvec), double2float);
	return fvec;
}
