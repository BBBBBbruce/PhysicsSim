#include "DynamicObj.h"


DynamicObj::DynamicObj()
{
}

/*
DynamicObj::DynamicObj(string n, string path, float m)
{
	name = n;
	loadingpath = path;
	mass = m;
	position = Eigen::Vector3f{ 0.,0.,0. };
	velocity = Eigen::Vector3f{ 0.,0.,0. };
	
}*/

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
	Eigen::MatrixXd vel, 
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

Eigen::MatrixXd DynamicObj::get_velocity()
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

void DynamicObj::updatestate(Eigen::Vector3d pos, Eigen::Vector3d v, bool collide, float t, int seq)
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

void DynamicObj::displayinfo()
{	
	cout << endl;
	cout << "name: " << name << endl;
	//cout << "path: " << loadingpath << endl;
	//cout << "position: " << glm::to_string(position) << endl;
	//cout << "scale: " << glm::to_string(scale) << endl;
	//cout << "rotation: " << glm::to_string(rotation) << endl;
	//cout << "velocity: " << glm::to_string(velocity) << endl;
	cout << "mass: " << mass << endl;
	cout << endl;
}

json DynamicObj::tojson()
{
	json j;
	j["type"] = "dynamic";
	//j["path"] = loadingpath;
	//j["position"] = { position.x(),position.y(),position.z() };
	//j["rotation"] = { rotation.x(), rotation.y(), rotation.z() };
	//j["scale"] = { scale.x(),scale.y(),scale.z() };
	//j["velocity"] = { velocity.x(),velocity.y(),velocity.z() };
	j["mass"] = mass;
	return j;
}

void DynamicObj::writemsh(string p, string v)
{
	igl::writeMSH(p, position, Tri, tetrahedral, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
	igl::writeMSH(v, velocity, Tri, tetrahedral, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
}


void DynamicObj::ToViewer(Eigen::MatrixXd& vertices, Eigen::MatrixXi& faces)
{
	using namespace Eigen;
	int row = tetrahedral.rows();

	MatrixXd V(row * 4, 3);
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

tuple<Eigen::MatrixXd, Eigen::MatrixXi> DynamicObj::Get_ViewMatrix()
{
	using namespace Eigen;
	int row = tetrahedral.rows();

	MatrixXd V(row * 4, 3);
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
