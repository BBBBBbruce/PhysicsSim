#include "DynamicObj.h"

using namespace Eigen;

//Vector3f gravity = {0.f, 10.f, 0.f};

//Eigen::Matrix4f rotate_eigen_api(Vector3f omega) {
//	Affine3f rx(AngleAxisf(omega[0], Vector3f::UnitX()));
//	Matrix4f rotationx = rx.matrix();
//	Affine3f ry(AngleAxisf(omega[1], Vector3f::UnitY()));
//	Matrix4f rotationy = ry.matrix();
//	Affine3f rz(AngleAxisf(omega[2], Vector3f::UnitZ()));
//	Matrix4f rotationz = rz.matrix();
//	return rotationx * rotationy * rotationz;
//}

//void DynamicObj::rotate(float t, Vector3f rotating_point) {
//	Affine3f tl(Translation3f(-rotating_point[0], -rotating_point[1], -rotating_point[2]));
//	Matrix4f translation = tl.matrix();
//	Affine3f tlb(Translation3f(rotating_point[0], rotating_point[1], rotating_point[2]));
//	Matrix4f translation_back = tlb.matrix();
//
//	MatrixXf pos_tmp = position;
//	pos_tmp.conservativeResize(pos_tmp.rows(), 4);// Make V n*4 matrix
//	pos_tmp.col(3).setOnes();
//	pos_tmp = (translation_back * rotate_eigen_api(angular_velocity*t) * translation * pos_tmp.transpose()).transpose();
//	position = MatrixXf(pos_tmp.rows(), 3);
//	position.col(0) = pos_tmp.col(0);
//	position.col(1) = pos_tmp.col(1);
//	position.col(2) = pos_tmp.col(2);
//
//	//rotate gravity_centre
//}

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
	linear_velocity = vel;
	mass = m;
	angular_velocity = Vector3f(0.0f, 0.0f, 0.0f);
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

void DynamicObj::updatestate(Eigen::Vector3f pos, Eigen::Vector3f v, bool collide, float t, int seq)
{
	//if (collide) {
	//	linear_velocity *= -0.8;
	//	cout << seq <<endl;
	//	//BUG: should update postion as well, for simplicity, stay the same
	//}
	//else {
	//	
	//	position.rowwise() += pos.transpose();
	//	position += linear_velocity * t;
	//	linear_velocity.rowwise() += v.transpose();
	//	cout << seq <<endl;
	//}
	//simulate_linear(pos, v, collide, t);
	//simulate_angular(t);
}

void DynamicObj::update_state(Eigen::MatrixXf x, Eigen::Vector3f v, Eigen::Vector3f w, Eigen::Vector3f cm)
{
	position = x;
	linear_velocity = v;
	angular_velocity = w;
	mass_centre = cm;
	
}

void DynamicObj::writemsh(string p, string v)
{	
	vector<Eigen::MatrixXd> xf, trif, tetf;
	xf   = cast2double(XF);
	trif = cast2double(TriF);
	tetf = cast2double(TetF);
	igl::writeMSH(p, float2double(position), Tri, tetrahedral, TriTag, TetTag, XFields, xf, EFields, trif, tetf);
	igl::writeMSH(v, float2double(linear_velocity), Tri, tetrahedral, TriTag, TetTag, XFields, xf, EFields, trif, tetf);
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
