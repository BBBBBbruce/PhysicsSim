#include "StaticObj.h"

StaticObj::StaticObj()
{
}

StaticObj::StaticObj(string n, 
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
	std::string path
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
	loadingpath = path;
}


string StaticObj::get_loadingpath()
{
	return loadingpath;
}


void StaticObj::ToViewer(Eigen::MatrixXf& vertices, Eigen::MatrixXi& faces)
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

tuple<Eigen::MatrixXf, Eigen::MatrixXi> StaticObj::Get_ViewMatrix()
{
	using namespace Eigen;
	int row = tetrahedral.rows();

	MatrixXf V(row * 4, 3);
	MatrixXi F(row * 4, 3);

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
	return { V,F };
}

