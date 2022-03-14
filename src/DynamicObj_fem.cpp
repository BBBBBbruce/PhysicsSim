#include "..\inc\DynamicObj_fem.h"

DynamicObj_fem::DynamicObj_fem()
{
}

DynamicObj_fem::DynamicObj_fem(string n,
	Eigen::MatrixXd pos,
	Eigen::MatrixXi tet,
	Eigen::MatrixXi tri,
	Eigen::MatrixXi tritag,
	Eigen::MatrixXi tettag,
	std::vector<std::string> xfields,
	std::vector<std::string> efields,
	std::vector<Eigen::MatrixXd> xf,
	std::vector<Eigen::MatrixXd> trif,
	std::vector<Eigen::MatrixXd> tetf,
	Eigen::Vector3d linear_vel,
	Eigen::Vector3d angular_vel,
	Eigen::MatrixXd Deform_vel,
	Eigen::MatrixXd rigid_pos,
	MatrixXd m_vec,
	double m_sum,
	MatrixXd ym
){
	name = n;
	position = pos;
	tetrahedrons = tet;
	Tri = tri;
	TriTag = tritag;
	TetTag = tettag;
	XFields = xfields;
	EFields = efields;
	XF = xf;
	TriF = trif;
	TetF = tetf;
	linear_velocity = linear_vel;
	angular_velocity = angular_vel;
	deform_velocity = Deform_vel;
	RigidPos = rigid_pos;
	mass_centre = find_cm();
	mass_vec = m_vec;
	mass_sum = m_sum;
	YModulus = ym;

}

Eigen::Vector3d DynamicObj_fem::get_linear_velocity()
{
	return linear_velocity;
}

Eigen::MatrixXi DynamicObj_fem::get_tetrahedrons()
{
	return tetrahedrons;
}

Eigen::Vector3d DynamicObj_fem::get_angular_velocity()
{
	return angular_velocity;
}

Eigen::MatrixXd DynamicObj_fem::get_deformation_velocity()
{
	return deform_velocity;
}

Eigen::MatrixXd DynamicObj_fem::get_rigid_position()
{
	return RigidPos;
}

double DynamicObj_fem::get_youngs_modulus()
{
	return Young;
}

MatrixXd DynamicObj_fem::get_mass_vec()
{
	return mass_vec;
}

double DynamicObj_fem::get_mass_sum()
{
	return mass_sum;
}

double DynamicObj_fem::get_poisson_ratio()
{
	return Poisson;
}

MatrixXd DynamicObj_fem::get_YModulus()
{
	return YModulus;
}

Eigen::Vector3d DynamicObj_fem::get_cm()
{
	return mass_centre;
}

void DynamicObj_fem::update_state(Eigen::MatrixXd x, Eigen::Vector3d v, Eigen::Vector3d w)
{
	//save for later
}

void DynamicObj_fem::save_current_state(string p)
{
	//properties need to be saved as files:

}

void DynamicObj_fem::save_debug_info()
{
}
