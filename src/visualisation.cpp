#include "visualisation.h"
#include "Objects.h"

void render(string path)
{
    Eigen::MatrixXd X, B;
    Eigen::MatrixXi Tri;
    Eigen::MatrixXi Tet;
    Eigen::VectorXi TriTag;
    Eigen::VectorXi TetTag;

    std::vector<std::string> XFields;
    std::vector<std::string> EFields;

    std::vector<Eigen::MatrixXd> XF;
    std::vector<Eigen::MatrixXd> TriF;
    std::vector<Eigen::MatrixXd> TetF;
    using namespace Eigen;

    igl::readMSH(path, X, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);

    // Compute barycenters
    //igl::barycenter(X, Tet, B);

    //std::cout << B << std::endl;
    // Plot the generated mesh
    igl::opengl::glfw::Viewer viewer;

    int row = Tet.rows();

    MatrixXd Vt(row * 4, 3);
    MatrixXi F(row * 4, 3);

    // list the tetrahedrals

    for (unsigned i = 0; i < row; ++i)
    {
        Vt.row(i * 4 + 0) = X.row(Tet(i, 0));
        Vt.row(i * 4 + 1) = X.row(Tet(i, 1));
        Vt.row(i * 4 + 2) = X.row(Tet(i, 2));
        Vt.row(i * 4 + 3) = X.row(Tet(i, 3));

        F.row(i * 4 + 0) << (i * 4) + 0, (i * 4) + 1, (i * 4) + 3;
        F.row(i * 4 + 1) << (i * 4) + 0, (i * 4) + 2, (i * 4) + 1;
        F.row(i * 4 + 2) << (i * 4) + 3, (i * 4) + 2, (i * 4) + 0;
        F.row(i * 4 + 3) << (i * 4) + 1, (i * 4) + 2, (i * 4) + 3;

    }
    //V_temp = V_temp * 0.5;
    //cout << V << endl; 
    MatrixXf V = double2float(Vt);

    V.conservativeResize(V.rows(), 4);// Make V n*4 matrix
    V.col(3).setOnes();

    // translation, rotation, scaling matrices
    Affine3f tl(Translation3f(2.0, 2.0, 2.0));
    Matrix4f translation = tl.matrix();
    Affine3f sc(AlignedScaling3f(0.5, 0.5, 0.5));
    Matrix4f scale = sc.matrix();
    Affine3f rx(AngleAxisf(0.5, Vector3f::UnitX()));
    Matrix4f rotationx = rx.matrix();
    Affine3f ry(AngleAxisf(0.5, Vector3f::UnitY()));
    Matrix4f rotationy = ry.matrix();
    Affine3f rz(AngleAxisf(0.5, Vector3f::UnitZ()));
    Matrix4f rotationz = rz.matrix();

    cout << translation * scale << endl;
    //cout << V << endl;
    //delete last column

    MatrixXf V_tmp = translation * rotationx * rotationy * rotationz * scale * V.transpose();
    V = V_tmp.transpose();

    MatrixXf _V(V.rows(), 3);
    _V.col(0) = V.col(0);
    _V.col(1) = V.col(1);
    _V.col(2) = V.col(2);
    
    viewer.data().set_mesh(float2double(_V), F);
    viewer.data().set_face_based(true);
    viewer.launch();
}

void SaveScene(string scene, string folder)
{

}

string padseq(int seq) {
    if (seq < 10)
        return "000" + to_string(seq);
    else if (seq < 100)
        return "00" + to_string(seq);
    else if (seq < 1000)
        return "0" + to_string(seq);
    else
        return to_string(seq);
}
