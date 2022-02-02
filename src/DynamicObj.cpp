#include "DynamicObj.h"

using namespace Eigen;

DynamicObj::DynamicObj()
{
}

DynamicObj::DynamicObj(string n, 
	Eigen::MatrixXf pos, 
	Eigen::MatrixXi tet, 
	Eigen::MatrixXi tri, 
	Eigen::MatrixXi tritag, 
	Eigen::MatrixXi tettag, 
	std::vector<std::string> xfields, 
	std::vector<std::string> efields, 
	std::vector<Eigen::MatrixXf> xf, 
	std::vector<Eigen::MatrixXf> trif, 
	std::vector<Eigen::MatrixXf> tetf, 
	Eigen::Vector3f vel, 
	Eigen::Vector3f angular_vel,
	float m
	)
{
	name = n;
	position = pos;
	tetrahedral = tet;
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

Eigen::Vector3f DynamicObj::get_linear_velocity()
{
	return linear_velocity;
}

Eigen::MatrixXi DynamicObj::get_tetrahedrons()
{
	return tetrahedral;
}

Eigen::Vector3f DynamicObj::get_angular_velocity()
{
	return angular_velocity;
}

float DynamicObj::get_mass()
{
	return mass;
}

Eigen::Vector3f DynamicObj::get_cm()
{
	return mass_centre;
}

void DynamicObj::update_state(Eigen::MatrixXf x, Eigen::Vector3f v, Eigen::Vector3f w)
{
	position = x;
	linear_velocity = v;
	angular_velocity = w;
	
}

void DynamicObj::writemsh(string p)
{	
	vector<Eigen::MatrixXd> xf, trif, tetf;
	xf   = cast2double(XF);
	trif = cast2double(TriF);
	tetf = cast2double(TetF);
	igl::writeMSH(p, float2double(position), Tri, tetrahedral, TriTag, TetTag, XFields, xf, EFields, trif, tetf);
}


void DynamicObj::ToViewer(Eigen::MatrixXf& vertices, Eigen::MatrixXi& faces)
{
	using namespace Eigen;
	int row = tetrahedral.rows();

	MatrixXf V(row * 4, 3);
	MatrixXi F(row * 4, 3);

	// list the tetrahedrals

	for (unsigned i = 0; i < row; ++i)
	{
		V.row(i * 4 + 0) = position.row(tetrahedral(i, 0));
		V.row(i * 4 + 1) = position.row(tetrahedral(i, 1));
		V.row(i * 4 + 2) = position.row(tetrahedral(i, 2));
		V.row(i * 4 + 3) = position.row(tetrahedral(i, 3));

		F.row(i * 4 + 0) << (i * 4) + 0, (i * 4) + 1, (i * 4) + 3;
		F.row(i * 4 + 1) << (i * 4) + 0, (i * 4) + 2, (i * 4) + 1;
		F.row(i * 4 + 2) << (i * 4) + 3, (i * 4) + 2, (i * 4) + 0;
		F.row(i * 4 + 3) << (i * 4) + 1, (i * 4) + 2, (i * 4) + 3;

	}
	vertices << V;
	faces << F;

}

tuple<Eigen::MatrixXf, Eigen::MatrixXi> DynamicObj::Get_ViewMatrix()
{
	using namespace Eigen;
	int row = tetrahedral.rows();

	MatrixXf V(row * 4, 3);
	MatrixXi F(row * 4, 3);

	// list the tetrahedrals

	for (unsigned i = 0; i < row; ++i)
	{
		V.row(i * 4 + 0) = position.row(tetrahedral(i, 0));
		V.row(i * 4 + 1) = position.row(tetrahedral(i, 1));
		V.row(i * 4 + 2) = position.row(tetrahedral(i, 2));
		V.row(i * 4 + 3) = position.row(tetrahedral(i, 3));

		F.row(i * 4 + 0) << (i * 4) + 0, (i * 4) + 1, (i * 4) + 3;
		F.row(i * 4 + 1) << (i * 4) + 0, (i * 4) + 2, (i * 4) + 1;
		F.row(i * 4 + 2) << (i * 4) + 3, (i * 4) + 2, (i * 4) + 0;
		F.row(i * 4 + 3) << (i * 4) + 1, (i * 4) + 2, (i * 4) + 3;

	}
	return {V,F};
}

Vector3f DynamicObj::find_cm()
{	
	//Vector3f out = position.colwise().mean();	
	return position.colwise().mean();
}


