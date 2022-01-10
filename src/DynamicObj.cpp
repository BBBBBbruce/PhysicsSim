#include "DynamicObj.h"


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
	Eigen::MatrixXf vel, 
	float m)
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
	velocity = vel;
	mass = m;
}

Eigen::MatrixXf DynamicObj::get_velocity()
{
	return velocity;
}

Eigen::MatrixXi DynamicObj::get_tetrahedrons()
{
	return tetrahedral;
}

float DynamicObj::get_mass()
{
	return mass;
}

void DynamicObj::updatestate(Eigen::Vector3f pos, Eigen::Vector3f v, bool collide, float t, int seq)
{
	if (collide) {
		velocity *= -0.8;
		cout << seq <<endl;
		//BUG: should update postion as well, for simplicity, stay the same
	}
	else {
		
		position.rowwise() += pos.transpose();
		position += velocity * t;
		velocity.rowwise() += v.transpose();
		cout << seq <<endl;
	}
}

void DynamicObj::writemsh(string p, string v)
{	
	vector<Eigen::MatrixXd> xf, trif, tetf;
	xf   = cast2double(XF);
	trif = cast2double(TriF);
	tetf = cast2double(TetF);
	igl::writeMSH(p, float2double(position), Tri, tetrahedral, TriTag, TetTag, XFields, xf, EFields, trif, tetf);
	igl::writeMSH(v, float2double(velocity), Tri, tetrahedral, TriTag, TetTag, XFields, xf, EFields, trif, tetf);
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
