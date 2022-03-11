#include "DynamicObj.h"

using namespace Eigen;

DynamicObj::DynamicObj()
{
}

DynamicObj::DynamicObj(string n, 
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
	Eigen::Vector3d vel, 
	Eigen::Vector3d angular_vel,
	double m
	)
{
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
	//std::cout << "here is problem" << std::endl;
	linear_velocity = vel;
	//std::cout << "here is problem" << std::endl;
	mass_centre = find_cm();
	//std::cout << "here is problem" << std::endl;
	mass = m;
	angular_velocity = angular_vel;
	//translation_velocity = vel_t;
}

Eigen::Vector3d DynamicObj::get_linear_velocity()
{
	return linear_velocity;
}

Eigen::MatrixXi DynamicObj::get_tetrahedrons()
{
	return tetrahedrons;
}

Eigen::Vector3d DynamicObj::get_angular_velocity()
{
	return angular_velocity;
}

double DynamicObj::get_mass()
{
	return mass;
}

Eigen::Vector3d DynamicObj::get_cm()
{
	return mass_centre;
}

void DynamicObj::update_state(Eigen::MatrixXd x, Eigen::Vector3d v, Eigen::Vector3d w)
{
	position = x;
	linear_velocity = v;
	angular_velocity = w;
	
}

void DynamicObj::writemsh(string p)
{	
	//vector<Eigen::MatrixXd> xf, trif, tetf;
	//xf   = cast2double(XF);
	//trif = cast2double(TriF);
	//tetf = cast2double(TetF);
	//igl::writeMSH(p, float2double(position), Tri, tetrahedral, TriTag, TetTag, XFields, xf, EFields, trif, tetf);
	igl::writeMSH(p, position, Tri, tetrahedrons, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
}

void DynamicObj::ToViewer(Eigen::MatrixXd& vertices, Eigen::MatrixXi& faces)
{
	using namespace Eigen;
	int row = tetrahedrons.rows();

	MatrixXd V(row * 4, 3);
	MatrixXi F(row * 4, 3);

	// list the tetrahedrals

	for (unsigned i = 0; i < row; ++i)
	{
		V.row(i * 4 + 0) = position.row(tetrahedrons(i, 0));
		V.row(i * 4 + 1) = position.row(tetrahedrons(i, 1));
		V.row(i * 4 + 2) = position.row(tetrahedrons(i, 2));
		V.row(i * 4 + 3) = position.row(tetrahedrons(i, 3));

		F.row(i * 4 + 0) << (i * 4) + 0, (i * 4) + 1, (i * 4) + 3;
		F.row(i * 4 + 1) << (i * 4) + 0, (i * 4) + 2, (i * 4) + 1;
		F.row(i * 4 + 2) << (i * 4) + 3, (i * 4) + 2, (i * 4) + 0;
		F.row(i * 4 + 3) << (i * 4) + 1, (i * 4) + 2, (i * 4) + 3;

	}
	vertices << V;
	faces << F;

}

tuple<Eigen::MatrixXd, Eigen::MatrixXi> DynamicObj::Get_ViewMatrix()
{
	using namespace Eigen;
	int row = tetrahedrons.rows();

	MatrixXd V(row * 4, 3);
	MatrixXi F(row * 4, 3);

	// list the tetrahedrals

	for (unsigned i = 0; i < row; ++i)
	{
		V.row(i * 4 + 0) = position.row(tetrahedrons(i, 0));
		V.row(i * 4 + 1) = position.row(tetrahedrons(i, 1));
		V.row(i * 4 + 2) = position.row(tetrahedrons(i, 2));
		V.row(i * 4 + 3) = position.row(tetrahedrons(i, 3));

		F.row(i * 4 + 0) << (i * 4) + 0, (i * 4) + 1, (i * 4) + 3;
		F.row(i * 4 + 1) << (i * 4) + 0, (i * 4) + 2, (i * 4) + 1;
		F.row(i * 4 + 2) << (i * 4) + 3, (i * 4) + 2, (i * 4) + 0;
		F.row(i * 4 + 3) << (i * 4) + 1, (i * 4) + 2, (i * 4) + 3;

	}
	return {V,F};
}

Vector3d DynamicObj::find_cm()
{	
	//Vector3d out = position.colwise().mean();	
	return position.colwise().mean();
}


