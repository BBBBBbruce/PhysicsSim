#include "utiles.h"


using namespace Eigen;
using namespace std;

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
	std::transform(vec.begin(), vec.end(), std::back_inserter(dvec), float2double);
	return dvec;
}

vector<Eigen::MatrixXf> cast2float(const vector<Eigen::MatrixXd>& vec)
{
	std::vector<Eigen::MatrixXf> fvec;
	fvec.reserve(vec.size());
	std::transform(vec.begin(), vec.end(), std::back_inserter(fvec), double2float);
	return fvec;
}

void make_dir_win(string targetpath, int seq) {
	if (seq < 10)
		_mkdir((targetpath + "000" + to_string(seq)).c_str());
	else if (seq < 100)
		_mkdir((targetpath + "00" + to_string(seq)).c_str());
	else if (seq < 1000)
		_mkdir((targetpath + "0" + to_string(seq)).c_str());
	else
		_mkdir((targetpath + to_string(seq)).c_str());
}

Eigen::Matrix4d rotate_eigen_api(Vector3d theta) {
    Affine3d rx(AngleAxisd(theta[0], Vector3d::UnitX()));
    Matrix4d rotationx = rx.matrix();
    Affine3d ry(AngleAxisd(theta[1], Vector3d::UnitY()));
    Matrix4d rotationy = ry.matrix();
    Affine3d rz(AngleAxisd(theta[2], Vector3d::UnitZ()));
    Matrix4d rotationz = rz.matrix();
    return rotationx * rotationy * rotationz;
}

void rotate(MatrixXd& x, Vector3d& theta, Vector3d& cm) {
    // define theta: 
    // Right hand rule: (+) --> counter-clockwise, (-) --> clockwise
    // x rotates about Vector3d::UnitX(), y rotates about Vector3f::UnitY(), z rotates about Vector3f::UnitZ()
    Affine3d tl(Translation3d(-cm[0], -cm[1], -cm[2]));
    Matrix4d translation = tl.matrix();
    Affine3d tlb(Translation3d(cm[0], cm[1], cm[2]));
    Matrix4d translation_back = tlb.matrix();

    MatrixXd pos_tmp = x;
    pos_tmp.conservativeResize(pos_tmp.rows(), 4);// Make V n*4 matrix
    pos_tmp.col(3).setOnes();
    pos_tmp = (translation_back * rotate_eigen_api(theta) * translation * pos_tmp.transpose()).transpose();
    x = MatrixXd(pos_tmp.rows(), 3);
    x.col(0) = pos_tmp.col(0);
    x.col(1) = pos_tmp.col(1);
    x.col(2) = pos_tmp.col(2);
}

Eigen::Vector3d rowwise_projection(Eigen::Vector3d vec, Eigen::Vector3d dir_vec) {
    return dir_vec.normalized() * vec.dot(dir_vec);
}

Eigen::MatrixXd projection(Eigen::MatrixXd matrix, Eigen::Vector3d dir_vec) {
    auto dots = matrix * dir_vec;
    MatrixXd output = dir_vec.normalized().replicate(matrix.rows(), 1);

    output = output.array() * dots.replicate(1, 3).array();// debug til runtime
    return output;
}

Eigen::Vector3d project_cos(Eigen::Vector3d vec, Eigen::Vector3d dir_vec) {
    auto dots = vec.transpose() * dir_vec;
    Vector3d output = dir_vec.normalized() * dots;
    return output;
}

Eigen::Vector3d project_sin(Eigen::Vector3d vec, Eigen::Vector3d dir_vec) {
    auto dots = vec.transpose() * dir_vec;
    Vector3d output = dir_vec.normalized() * dots;
    return vec - output;
}

float TetrahedronElementVolume(MatrixXd tet_vertices) {

    MatrixXd new_mat(tet_vertices.rows(), tet_vertices.cols() + 1);
    new_mat << Matrix<double, 4, 1>::Ones(), tet_vertices;
    //new_mat.setOnes();

    //cout << new_mat << endl;
    return new_mat.determinant();
}

MatrixXi tet_re_order(MatrixXi tet, MatrixXd p1, MatrixXd p2, MatrixXd p3, MatrixXd p4) {

    vector<int> indices;
    indices.push_back(tet(0, 0));
    indices.push_back(tet(0, 1));
    indices.push_back(tet(0, 2));
    indices.push_back(tet(0, 3));
    //cout << "tet" << endl << tet << endl;

    map<int, MatrixXd> vertex_index;
    //initializing
    vertex_index[tet(0, 0)] = p1;
    vertex_index[tet(0, 1)] = p2;
    vertex_index[tet(0, 2)] = p3;
    vertex_index[tet(0, 3)] = p4;

    Matrix<double, 4, 3> teti;
    teti << p1, p2, p3, p4;
    auto rng = std::default_random_engine{};

    //cout << "tet: " << tet << " volume is "<< TetrahedronElementVolume(teti) <<endl;


    while (TetrahedronElementVolume(teti) < 0) {

        std::shuffle(std::begin(indices), std::end(indices), rng);
        teti.row(indices[0]) = vertex_index[indices[0]];
        teti.row(indices[1]) = vertex_index[indices[1]];
        teti.row(indices[2]) = vertex_index[indices[2]];
        teti.row(indices[3]) = vertex_index[indices[3]];
    }
    Matrix<int, 1, 4> out;
    out << indices[0], indices[1], indices[2], indices[3];
    //cout << "tet: " << out << " volume is " << TetrahedronElementVolume(teti) << endl;
    return out;
}

MatrixXd TetrahedronElementStiffness(MatrixXd YModulus, MatrixXd vertices) {
    double x1 = vertices(0, 0);
    double y1 = vertices(0, 1);
    double z1 = vertices(0, 2);
    double x2 = vertices(1, 0);
    double y2 = vertices(1, 1);
    double z2 = vertices(1, 2);
    double x3 = vertices(2, 0);
    double y3 = vertices(2, 1);
    double z3 = vertices(2, 2);
    double x4 = vertices(3, 0);
    double y4 = vertices(3, 1);
    double z4 = vertices(3, 2);

    Matrix<double, 3, 3> mbeta1;
    mbeta1 << 1, y2, z2, 1, y3, z3, 1, y4, z4;
    Matrix<double, 3, 3> mbeta2;
    mbeta2 << 1, y1, z1, 1, y3, z3, 1, y4, z4;
    Matrix<double, 3, 3> mbeta3;
    mbeta3 << 1, y1, z1, 1, y2, z2, 1, y4, z4;
    Matrix<double, 3, 3> mbeta4;
    mbeta4 << 1, y1, z1, 1, y2, z2, 1, y3, z3;

    /*cout << "mbeta1: " << endl << mbeta1 << endl;
    cout << "mbeta2: " << endl << mbeta2 << endl;
    cout << "mbeta3: " << endl << mbeta3 << endl;
    cout << "mbeta4: " << endl << mbeta4 << endl;*/


    Matrix<double, 3, 3> mgamma1;
    mgamma1 << 1, x2, z2, 1, x3, z3, 1, x4, z4;
    Matrix<double, 3, 3> mgamma2;
    mgamma2 << 1, x1, z1, 1, x3, z3, 1, x4, z4;
    Matrix<double, 3, 3> mgamma3;
    mgamma3 << 1, x1, z1, 1, x2, z2, 1, x4, z4;
    Matrix<double, 3, 3> mgamma4;
    mgamma4 << 1, x1, z1, 1, x2, z2, 1, x3, z3;

    /* cout << "mgamma1: " << endl << mgamma1 << endl;
     cout << "mgamma2: " << endl << mgamma2 << endl;
     cout << "mgamma3: " << endl << mgamma3 << endl;
     cout << "mgamma4: " << endl << mgamma4 << endl; */

    Matrix<double, 3, 3> mdelta1;
    mdelta1 << 1, x2, y2, 1, x3, y3, 1, x4, y4;
    Matrix<double, 3, 3> mdelta2;
    mdelta2 << 1, x1, y1, 1, x3, y3, 1, x4, y4;
    Matrix<double, 3, 3> mdelta3;
    mdelta3 << 1, x1, y1, 1, x2, y2, 1, x4, y4;
    Matrix<double, 3, 3> mdelta4;
    mdelta4 << 1, x1, y1, 1, x2, y2, 1, x3, y3;

    /*cout << "mdelta1: " << endl << mdelta1 << endl;
    cout << "mdelta2: " << endl << mdelta2 << endl;
    cout << "mdelta3: " << endl << mdelta3 << endl;
    cout << "mdelta4: " << endl << mdelta4 << endl;*/

    float beta1 = -1 * mbeta1.determinant();
    float beta2 = mbeta2.determinant();
    float beta3 = -1 * mbeta3.determinant();
    float beta4 = mbeta4.determinant();

    //cout << "beta1: " << beta1 << endl;
    //cout << "beta2: " << beta2 << endl;
    //cout << "beta3: " << beta3 << endl;
    //cout << "beta4: " << beta4 << endl;

    float gamma1 = mgamma1.determinant();
    float gamma2 = -1 * mgamma2.determinant();
    float gamma3 = mgamma3.determinant();
    float gamma4 = -1 * mgamma4.determinant();

    //cout << "gamma1: " << gamma1 << endl;
    //cout << "gamma2: " << gamma2 << endl;
    //cout << "gamma3: " << gamma3 << endl;
    //cout << "gamma4: " << gamma4 << endl;

    float delta1 = -1 * mdelta1.determinant();
    float delta2 = mdelta2.determinant();
    float delta3 = -1 * mdelta3.determinant();
    float delta4 = mdelta4.determinant();

    //cout << "delta1: " << delta1 << endl;
    //cout << "delta2: " << delta2 << endl;
    //cout << "delta3: " << delta3 << endl;
    //cout << "delta4: " << delta4 << endl;

    Matrix<double, 6, 12> B;
    B << beta1, 0, 0, beta2, 0, 0, beta3, 0, 0, beta4, 0, 0,
        0, gamma1, 0, 0, gamma2, 0, 0, gamma3, 0, 0, gamma4, 0,
        0, 0, delta1, 0, 0, delta2, 0, 0, delta3, 0, 0, delta4,
        gamma1, beta1, 0, gamma2, beta2, 0, gamma3, beta3, 0, gamma4, beta4, 0,
        0, delta1, gamma1, 0, delta2, gamma2, 0, delta3, gamma3, 0, delta4, gamma4,
        delta1, 0, beta1, delta2, 0, beta2, delta3, 0, beta3, delta4, 0, beta4;
    double V = TetrahedronElementVolume(vertices);
    //cout << "V: " << endl << V << endl;
    B /= (6 * V);
    //cout << "YModulus" << endl << YModulus << endl;
    MatrixXd out = V * B.transpose() * YModulus * B;
    return out;
}

void TetrahedronAssemble(MatrixXd& K, MatrixXd k, MatrixXi indices) {
    int i = indices(0, 0);
    int j = indices(0, 1);
    int m = indices(0, 2);
    int n = indices(0, 3);
    //cout << i << j << m << n << endl;
    //cout << k.diagonal() << endl;
    K(3 * i, 3 * i) += k(0, 0);
    K(3 * i, 3 * i + 1) += k(0, 1);
    K(3 * i, 3 * i + 2) += k(0, 2);
    K(3 * i, 3 * j) += k(0, 3);
    K(3 * i, 3 * j + 1) += k(0, 4);
    K(3 * i, 3 * j + 2) += k(0, 5);
    K(3 * i, 3 * m) += k(0, 6);
    K(3 * i, 3 * m + 1) += k(0, 7);
    K(3 * i, 3 * m + 2) += k(0, 8);
    K(3 * i, 3 * n) += k(0, 9);
    K(3 * i, 3 * n + 1) += k(0, 10);
    K(3 * i, 3 * n + 2) += k(0, 11);

    K(3 * i + 1, 3 * i) += k(1, 0);
    K(3 * i + 1, 3 * i + 1) += k(1, 1);
    K(3 * i + 1, 3 * i + 2) += k(1, 2);
    K(3 * i + 1, 3 * j) += k(1, 3);
    K(3 * i + 1, 3 * j + 1) += k(1, 4);
    K(3 * i + 1, 3 * j + 2) += k(1, 5);
    K(3 * i + 1, 3 * m) += k(1, 6);
    K(3 * i + 1, 3 * m + 1) += k(1, 7);
    K(3 * i + 1, 3 * m + 2) += k(1, 8);
    K(3 * i + 1, 3 * n) += k(1, 9);
    K(3 * i + 1, 3 * n + 1) += k(1, 10);
    K(3 * i + 1, 3 * n + 2) += k(1, 11);

    K(3 * i + 2, 3 * i) += k(2, 0);
    K(3 * i + 2, 3 * i + 1) += k(2, 1);
    K(3 * i + 2, 3 * i + 2) += k(2, 2);
    K(3 * i + 2, 3 * j) += k(2, 3);
    K(3 * i + 2, 3 * j + 1) += k(2, 4);
    K(3 * i + 2, 3 * j + 2) += k(2, 5);
    K(3 * i + 2, 3 * m) += k(2, 6);
    K(3 * i + 2, 3 * m + 1) += k(2, 7);
    K(3 * i + 2, 3 * m + 2) += k(2, 8);
    K(3 * i + 2, 3 * n) += k(2, 9);
    K(3 * i + 2, 3 * n + 1) += k(2, 10);
    K(3 * i + 2, 3 * n + 2) += k(2, 11);

    K(3 * j, 3 * i) += k(3, 0);
    K(3 * j, 3 * i + 1) += k(3, 1);
    K(3 * j, 3 * i + 2) += k(3, 2);
    K(3 * j, 3 * j) += k(3, 3);
    K(3 * j, 3 * j + 1) += k(3, 4);
    K(3 * j, 3 * j + 2) += k(3, 5);
    K(3 * j, 3 * m) += k(3, 6);
    K(3 * j, 3 * m + 1) += k(3, 7);
    K(3 * j, 3 * m + 2) += k(3, 8);
    K(3 * j, 3 * n) += k(3, 9);
    K(3 * j, 3 * n + 1) += k(3, 10);
    K(3 * j, 3 * n + 2) += k(3, 11);

    K(3 * j + 1, 3 * i) += k(4, 0);
    K(3 * j + 1, 3 * i + 1) += k(4, 1);
    K(3 * j + 1, 3 * i + 2) += k(4, 2);
    K(3 * j + 1, 3 * j) += k(4, 3);
    K(3 * j + 1, 3 * j + 1) += k(4, 4);
    K(3 * j + 1, 3 * j + 2) += k(4, 5);
    K(3 * j + 1, 3 * m) += k(4, 6);
    K(3 * j + 1, 3 * m + 1) += k(4, 7);
    K(3 * j + 1, 3 * m + 2) += k(4, 8);
    K(3 * j + 1, 3 * n) += k(4, 9);
    K(3 * j + 1, 3 * n + 1) += k(4, 10);
    K(3 * j + 1, 3 * n + 2) += k(4, 11);
    //
    K(3 * j + 2, 3 * i) += k(5, 0);
    K(3 * j + 2, 3 * i + 1) += k(5, 1);
    K(3 * j + 2, 3 * i + 2) += k(5, 2);
    K(3 * j + 2, 3 * j) += k(5, 3);
    K(3 * j + 2, 3 * j + 1) += k(5, 4);
    K(3 * j + 2, 3 * j + 2) += k(5, 5);
    K(3 * j + 2, 3 * m) += k(5, 6);
    K(3 * j + 2, 3 * m + 1) += k(5, 7);
    K(3 * j + 2, 3 * m + 2) += k(5, 8);
    K(3 * j + 2, 3 * n) += k(5, 9);
    K(3 * j + 2, 3 * n + 1) += k(5, 10);
    K(3 * j + 2, 3 * n + 2) += k(5, 11);

    K(3 * m, 3 * i) += k(6, 0);
    K(3 * m, 3 * i + 1) += k(6, 1);
    K(3 * m, 3 * i + 2) += k(6, 2);
    K(3 * m, 3 * j) += k(6, 3);
    K(3 * m, 3 * j + 1) += k(6, 4);
    K(3 * m, 3 * j + 2) += k(6, 5);
    K(3 * m, 3 * m) += k(6, 6);
    K(3 * m, 3 * m + 1) += k(6, 7);
    K(3 * m, 3 * m + 2) += k(6, 8);
    K(3 * m, 3 * n) += k(6, 9);
    K(3 * m, 3 * n + 1) += k(6, 10);
    K(3 * m, 3 * n + 2) += k(6, 11);

    K(3 * m + 1, 3 * i) += k(7, 0);
    K(3 * m + 1, 3 * i + 1) += k(7, 1);
    K(3 * m + 1, 3 * i + 2) += k(7, 2);
    K(3 * m + 1, 3 * j) += k(7, 3);
    K(3 * m + 1, 3 * j + 1) += k(7, 4);
    K(3 * m + 1, 3 * j + 2) = k(7, 5);
    K(3 * m + 1, 3 * m) += k(7, 6);
    K(3 * m + 1, 3 * m + 1) += k(7, 7);
    K(3 * m + 1, 3 * m + 2) += k(7, 8);
    K(3 * m + 1, 3 * n) += k(7, 9);
    K(3 * m + 1, 3 * n + 1) += k(7, 10);
    K(3 * m + 1, 3 * n + 2) += k(7, 11);

    K(3 * m + 2, 3 * i) += k(8, 0);
    K(3 * m + 2, 3 * i + 1) += k(8, 1);
    K(3 * m + 2, 3 * i + 2) += k(8, 2);
    K(3 * m + 2, 3 * j) += k(8, 3);
    K(3 * m + 2, 3 * j + 1) += k(8, 4);
    K(3 * m + 2, 3 * j + 2) += k(8, 5);
    K(3 * m + 2, 3 * m) += k(8, 6);
    K(3 * m + 2, 3 * m + 1) += k(8, 7);
    K(3 * m + 2, 3 * m + 2) += k(8, 8);
    K(3 * m + 2, 3 * n) += k(8, 9);
    K(3 * m + 2, 3 * n + 1) += k(8, 10);
    K(3 * m + 2, 3 * n + 2) += k(8, 11);

    K(3 * n, 3 * i) += k(9, 0);
    K(3 * n, 3 * i + 1) += k(9, 1);
    K(3 * n, 3 * i + 2) += k(9, 2);
    K(3 * n, 3 * j) += k(9, 3);
    K(3 * n, 3 * j + 1) += k(9, 4);
    K(3 * n, 3 * j + 2) += k(9, 5);
    K(3 * n, 3 * m) += k(9, 6);
    K(3 * n, 3 * m + 1) += k(9, 7);
    K(3 * n, 3 * m + 2) += k(9, 8);
    K(3 * n, 3 * n) += k(9, 9);
    K(3 * n, 3 * n + 1) += k(9, 10);
    K(3 * n, 3 * n + 2) += k(9, 11);

    K(3 * n + 1, 3 * i) += k(10, 0);
    K(3 * n + 1, 3 * i + 1) += k(10, 1);
    K(3 * n + 1, 3 * i + 2) += k(10, 2);
    K(3 * n + 1, 3 * j) += k(10, 3);
    K(3 * n + 1, 3 * j + 1) += k(10, 4);
    K(3 * n + 1, 3 * j + 2) += k(10, 5);
    K(3 * n + 1, 3 * m) += k(10, 6);
    K(3 * n + 1, 3 * m + 1) += k(10, 7);
    K(3 * n + 1, 3 * m + 2) += k(10, 8);
    K(3 * n + 1, 3 * n) += k(10, 9);
    K(3 * n + 1, 3 * n + 1) += k(10, 10);
    K(3 * n + 1, 3 * n + 2) += k(10, 11);

    K(3 * n + 2, 3 * i) += k(11, 0);
    K(3 * n + 2, 3 * i + 1) += k(11, 1);
    K(3 * n + 2, 3 * i + 2) += k(11, 2);
    K(3 * n + 2, 3 * j) += k(11, 3);
    K(3 * n + 2, 3 * j + 1) += k(11, 4);
    K(3 * n + 2, 3 * j + 2) += k(11, 5);
    K(3 * n + 2, 3 * m) += k(11, 6);
    K(3 * n + 2, 3 * m + 1) += k(11, 7);
    K(3 * n + 2, 3 * m + 2) += k(11, 8);
    K(3 * n + 2, 3 * n) += k(11, 9);
    K(3 * n + 2, 3 * n + 1) += k(11, 10);
    K(3 * n + 2, 3 * n + 2) += k(11, 11);
}

MatrixXd TetrahedronElementStresses(MatrixXd YModulus, MatrixXd vertices, MatrixXd u) {
    double x1 = vertices(0, 0);
    double y1 = vertices(0, 1);
    double z1 = vertices(0, 2);
    double x2 = vertices(1, 0);
    double y2 = vertices(1, 1);
    double z2 = vertices(1, 2);
    double x3 = vertices(2, 0);
    double y3 = vertices(2, 1);
    double z3 = vertices(2, 2);
    double x4 = vertices(3, 0);
    double y4 = vertices(3, 1);
    double z4 = vertices(3, 2);

    Matrix<double, 3, 3> mbeta1;
    mbeta1 << 1, y2, z2, 1, y3, z3, 1, y4, z4;
    Matrix<double, 3, 3> mbeta2;
    mbeta2 << 1, y1, z1, 1, y3, z3, 1, y4, z4;
    Matrix<double, 3, 3> mbeta3;
    mbeta3 << 1, y1, z1, 1, y2, z2, 1, y4, z4;
    Matrix<double, 3, 3> mbeta4;
    mbeta4 << 1, y1, z1, 1, y2, z2, 1, y3, z3;

    Matrix<double, 3, 3> mgamma1;
    mgamma1 << 1, x2, z2, 1, x3, z3, 1, x4, z4;
    Matrix<double, 3, 3> mgamma2;
    mgamma2 << 1, x1, z1, 1, x3, z3, 1, x4, z4;
    Matrix<double, 3, 3> mgamma3;
    mgamma3 << 1, x1, z1, 1, x2, z2, 1, x4, z4;
    Matrix<double, 3, 3> mgamma4;
    mgamma4 << 1, x1, z1, 1, x2, z2, 1, x3, z3;

    Matrix<double, 3, 3> mdelta1;
    mdelta1 << 1, x2, y2, 1, x3, y3, 1, x4, y4;
    Matrix<double, 3, 3> mdelta2;
    mdelta2 << 1, x1, y1, 1, x3, y3, 1, x4, y4;
    Matrix<double, 3, 3> mdelta3;
    mdelta3 << 1, x1, y1, 1, x2, y2, 1, x4, y4;
    Matrix<double, 3, 3> mdelta4;
    mdelta4 << 1, x1, y1, 1, x2, y2, 1, x3, y3;

    float beta1 = -1 * mbeta1.determinant();
    float beta2 = mbeta2.determinant();
    float beta3 = -1 * mbeta3.determinant();
    float beta4 = mbeta4.determinant();

    float gamma1 = mgamma1.determinant();
    float gamma2 = -1 * mgamma2.determinant();
    float gamma3 = mgamma3.determinant();
    float gamma4 = -1 * mgamma4.determinant();

    float delta1 = -1 * mdelta1.determinant();
    float delta2 = mdelta2.determinant();
    float delta3 = -1 * mdelta3.determinant();
    float delta4 = mdelta4.determinant();

    Matrix<double, 6, 12> B;
    B << beta1, 0, 0, beta2, 0, 0, beta3, 0, 0, beta4, 0, 0,
        0, gamma1, 0, 0, gamma2, 0, 0, gamma3, 0, 0, gamma4, 0,
        0, 0, delta1, 0, 0, delta2, 0, 0, delta3, 0, 0, delta4,
        gamma1, beta1, 0, gamma2, beta2, 0, gamma3, beta3, 0, gamma4, beta4, 0,
        0, delta1, gamma1, 0, delta2, gamma2, 0, delta3, gamma3, 0, delta4, gamma4,
        delta1, 0, beta1, delta2, 0, beta2, delta3, 0, beta3, delta4, 0, beta4;
    float V = TetrahedronElementVolume(vertices);
    B /= (6 * V);
    return YModulus * B * u;
}

Matrix<double, 1, 3> TetrahedronElementPStresses(Matrix<double, 6, 1> sigma) {
    double sg1 = sigma(0, 0);
    double sg2 = sigma(1, 0);
    double sg3 = sigma(2, 0);
    double sg4 = sigma(3, 0);
    double sg5 = sigma(4, 0);
    double sg6 = sigma(5, 0);
    double s1 = sg1 + sg2 + sg3;
    double s2 = sg1 * sg2 + sg1 * sg3 + sg2 * sg3 - sg4 * sg4 - sg5 * sg5 - sg6 * sg6;
    Matrix<double, 3, 3> ms3;
    ms3 << sg1, sg4, sg6, sg4, sg2, sg5, sg6, sg5, sg3;
    double s3 = ms3.determinant();
    Matrix<double, 1, 3> out;
    out << s1, s2, s3;
    return out;

}

void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
{
    unsigned int numRows = matrix.rows() - 1;
    unsigned int numCols = matrix.cols();

    if (rowToRemove < numRows)
        matrix.block(rowToRemove, 0, numRows - rowToRemove, numCols) = matrix.bottomRows(numRows - rowToRemove);

    matrix.conservativeResize(numRows, numCols);
}

void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
{
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols() - 1;

    if (colToRemove < numCols)
        matrix.block(0, colToRemove, numRows, numCols - colToRemove) = matrix.rightCols(numCols - colToRemove);

    matrix.conservativeResize(numRows, numCols);
}

MatrixXd insertRow(MatrixXd  matrix, int row, MatrixXd r) {
    MatrixXd out;
    out.resize(matrix.rows() + 1, matrix.cols());
    //cout << "Mat_in:" << endl << matrix << endl;
    ////cout << row << endl;
    ////cout << matrix.rows()<<endl;
    ////cout << matrix.cols() << endl;
    //cout << "mat1: " << endl << matrix.block(0, 0, row, matrix.cols()) << endl;
    //cout << "mat2: " << endl << r << endl;
    //cout << "mat3: " << endl << matrix.block(row, 0, matrix.rows() - row, matrix.cols()) << endl;

    out << matrix.block(0, 0, row, matrix.cols()), r, matrix.block(row, 0, matrix.rows() - row, matrix.cols());
    return out;
}

void Xd2csv(string fileName, MatrixXd  matrix)
{
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");

    ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(CSVFormat);
        file.close();
    }
}

void Xf2csv(string fileName, MatrixXd  matrix)
{
    const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");

    ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(CSVFormat);
        file.close();
    }
}

MatrixXd XfFromcsv(string fileToOpen)
{

    vector<double> matrixEntries;
    ifstream matrixDataFile(fileToOpen);
    string matrixRowString;
    string matrixEntry;
    int matrixRowNumber = 0;


    while (getline(matrixDataFile, matrixRowString))
    {
        stringstream matrixRowStringStream(matrixRowString);

        while (getline(matrixRowStringStream, matrixEntry, ','))
        {
            matrixEntries.push_back(stod(matrixEntry));
        }
        matrixRowNumber++;
    }
    return Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);

}

MatrixXd HookeLaw(MatrixXd epsilon, Matrix<double, 6, 6 > modulus) {
    Matrix<double, 6, 1> tmp;
    tmp << epsilon(0, 0), epsilon(1, 1), epsilon(2, 2),
        epsilon(0, 1), epsilon(1, 2), epsilon(0, 2);
    tmp = modulus * tmp;
    MatrixXd sigma(3, 3);
    sigma << tmp(0), tmp(3), tmp(5),
        tmp(3), tmp(1), tmp(4),
        tmp(5), tmp(4), tmp(2);
    return sigma;
}

MatrixXd apply_poisson(MatrixXd d, double p) {
    MatrixXd x = d.col(0);
    MatrixXd y = d.col(1);
    MatrixXd z = d.col(2);
    d.col(0) += -(y + z) * p;
    d.col(1) += -(x + z) * p;
    d.col(2) += -(x + y) * p;
    return d;
}

bool point_in_triangle(Matrix<double, 1, 3> p, Matrix<double, 1, 3> V0, Matrix<double, 1, 3> V1, Matrix<double, 1, 3> V2) {
    cout << "contact: " << endl << p << endl;
    double S = 0.5 * ((V0 - V1).cross(V0 - V2)).norm();
    double s1 = 0.5 * ((V0 - p).cross(V2 - p)).norm();
    double s2 = 0.5 * ((V1 - p).cross(V2 - p)).norm();
    double s3 = 0.5 * ((V1 - p).cross(V0 - p)).norm();
    double a = s1 / S;
    double b = s2 / S;
    double c = s3 / S;
    if (abs(a + b + c - 1) < 1.0e-05 && a > 0 && b > 0 && c > 0 && a < 1 && b < 1 && c < 1)
        return true;
    else
        return false;

}

tuple<bool, double> LinePlaneIntersection(
    Matrix<double, 1, 3> p,
    Matrix<double, 1, 3> V,
    Matrix<double, 1, 3> V1,
    Matrix<double, 1, 3> V2,
    Matrix<double, 1, 3> V3
) {
    //cout<<"p: "<<endl<<p<<endl;
    //cout<<"V: "<<endl<<V<<endl;
    //cout<<"V1: "<<endl<<V1<<endl;
    //cout<<"V2: "<<endl<<V2<<endl;
    //cout<<"V3: "<<endl<<V3<<endl;
    Matrix<double, 1, 3> line = p.row(0) - V;
    Matrix<double, 1, 3> x2 = V2 - V1;
    Matrix<double, 1, 3> x3 = V3 - V1;
    Matrix<double, 1, 3> normal = (x2.cross(x3)).normalized();

    //cout<<"normal: "<<endl<<normal<<endl;
    //cout<<"line: "<<endl<<line<<endl;
    if (normal.dot(line) == 0.0) {
        return { false,0.0 };
    }

    double d = normal.dot(V1);
    double x = (d - normal.dot(V)) / normal.dot(line);
    //cout<<"x: "<< x <<endl;
    Matrix<double, 1, 3> contact = V + line * x;
    double max_d = (contact - V).norm();
    if (max_d - line.norm() < 1.0e-05)
        if (point_in_triangle(contact, V1, V2, V3))
            // p = p0 + (p1 - p0) * s + (p2 - p0) * t
            return { true,max_d };
        else
            return { false,0.0 };
    else
        return { false,0.0 };
}

tuple<bool, double, int> self_collision_tet(Matrix<double, 4, 3> X, Matrix<double, 4, 3> P) {
    //cout<<"X: "<<endl<<X<<endl;
    //cout<<"P: "<<endl<<P<<endl;

    int index = -1;
    bool flag = false;
    double max_d = -1;
    auto [collide0, max_d0] = LinePlaneIntersection(P.row(0), X.row(0), X.row(1), X.row(2), X.row(3));
    //cout<<"collide0: "<<collide0<<endl;
    //cout<<"max_d0: "<<max_d0<<endl;
    if (collide0 && max_d0 > max_d) {
        flag = true;
        index = 0;
        max_d = max_d0;
    }
    auto [collide1, max_d1] = LinePlaneIntersection(P.row(1), X.row(1), X.row(0), X.row(2), X.row(3));
    //cout<<"collide1: "<<collide1<<endl;
    //cout<<"max_d1: "<<max_d1<<endl;
    if (collide1 && max_d1 > max_d) {
        flag = true;
        index = 0;
        max_d = max_d1;
    }
    auto [collide2, max_d2] = LinePlaneIntersection(P.row(2), X.row(2), X.row(1), X.row(0), X.row(3));
    //cout<<"collide2: "<<collide2<<endl;
    //cout<<"max_d2: "<<max_d2<<endl;
    if (collide2 && max_d2 > max_d) {
        flag = true;
        index = 0;
        max_d = max_d2;
    }
    auto [collide3, max_d3] = LinePlaneIntersection(P.row(3), X.row(3), X.row(1), X.row(2), X.row(0));
    //cout<<"collide3: "<<collide3<<endl;
    //cout<<"max_d3: "<<max_d3<<endl;
    if (collide3 && max_d3 > max_d) {
        flag = true;
        index = 0;
        max_d = max_d3;
    }

    return { flag,max_d,index };

}

MatrixXd mat2vec(MatrixXd mat, int row, int col) {
    //cout << "0" << endl << mat << endl;
    MatrixXd tmp = mat.transpose();
    //cout << "1" << endl << mat << endl;
    tmp.resize(row, col);
    //cout << "2" << endl << mat << endl;
    return tmp;
}

MatrixXd vec2mat(MatrixXd mat, int row, int col) {
    //cout << "0" << endl << mat << endl;
    mat.resize(col, row);
    return mat.transpose();
}