#include "GraphicsEngine.h"
#include<filesystem>
namespace fs = std::filesystem;


GraphicsEngine::GraphicsEngine()
{
    vector<StaticObj>StaticVec;
    vector<DynamicObj>DynamicVec;
}


GraphicsEngine::GraphicsEngine(string t_path)
{
	vector<StaticObj>StaticVec;
	vector<DynamicObj>DynamicVec;
	this->tpath = t_path;
}

vector<StaticObj> GraphicsEngine::getStaticObjs()
{
	return StaticVec;
}

vector<DynamicObj> GraphicsEngine::getDynamicObjs()
{
	return DynamicVec;
}

void GraphicsEngine::load_scene(string folder)
{
    string scene_in = folder + "\\Scene.json";
    std::ifstream ifs(scene_in);
    json j;

    try
    {
        j = json::parse(ifs);
    }
    catch (json::parse_error& ex)
    {
        std::cerr << "Yo, where the f* is your scene file "  << std::endl;
    }

    for (auto it = j["Objects"].begin(); it != j["Objects"].end(); ++it) {
        if (it.value()["type"] == "dynamic") {
            Eigen::MatrixXd X,V;
            Eigen::MatrixXi Tri, Tet;
            Eigen::VectorXi TriTag, TetTag;
            std::vector<std::string> XFields, EFields;
            std::vector<Eigen::MatrixXd> XF, TriF, TetF;
            igl::readMSH(it.value()["position_path"], X, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);

            DynamicObj dtmp(
                it.key(),
                double2float(X), Tet, Tri, TriTag, TetTag, XFields, EFields,
                cast2float(XF), cast2float(TriF), cast2float(TetF),
                double2float(V), it.value()["mass"]
            );
            DynamicVec.push_back(dtmp);
        }
        else if (it.value()["type"] == "static") {
            Eigen::MatrixXd X;
            Eigen::MatrixXi Tri, Tet;
            Eigen::VectorXi TriTag, TetTag;
            std::vector<std::string> XFields, EFields;
            std::vector<Eigen::MatrixXd> XF, TriF, TetF;

            igl::readMSH(it.value()["position_path"], X, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
            StaticObj stmp(
                it.key(),
                double2float(X), Tet, Tri, TriTag, TetTag, XFields, EFields,
                cast2float(XF), cast2float(TriF), cast2float(TetF), it.value()["position_path"]
            );
            StaticVec.push_back(stmp);
        }
    }
}

using namespace Eigen;

/*
    Eigen::MatrixXd V(VA.rows()+VB.rows(),VA.cols());
    V<<VA,VB;
    Eigen::MatrixXi F(FA.rows()+FB.rows(),FA.cols());
    F<<FA,(FB.array()+VA.rows());

    MatrixXi tmp = A;
    A.resize(A.rows() + B.rows(), NoChange);
    A << tmp, B;
    cout << A << "\n";
 */



void GraphicsEngine::save_scene(string t_folder, int seq)
{   

    igl::opengl::glfw::Viewer viewer;
    viewer.data().clear();
    Eigen::MatrixXf V;
    Eigen::MatrixXi F;
    V.resize(NoChange, 3);
    F.resize(NoChange, 3);
    //cout <<"no dimension bugs so far" << endl;
    viewer.data().clear();
    for (auto i = 0; i < DynamicVec.size(); i++) {
        auto [vtmp, ftmp] = DynamicVec[i].Get_ViewMatrix();
        //cout << "no dimension bugs so far" << endl;
        MatrixXf vtmp2 = V;
        MatrixXi ftmp2 = F;

        ftmp = ftmp.array() + V.rows();
        V.resize(vtmp.rows() + V.rows(), NoChange);
        V << vtmp2, vtmp;
        F.resize(ftmp.rows() + F.rows(), NoChange);
        F << ftmp2, ftmp;
    }

    for (auto i = 0; i < StaticVec.size(); i++) {

        auto [vtmp, ftmp] = StaticVec[i].Get_ViewMatrix();
        //cout << "no dimension bugs so far" << endl;
        MatrixXf vtmp2 = V;
        MatrixXi ftmp2 = F;
  

        ftmp = ftmp.array() + V.rows();
        V.resize(vtmp.rows() + V.rows(), 3);
        V << vtmp2, vtmp;
        F.resize(ftmp.rows() + F.rows(), 3);
        F << ftmp2, ftmp;
    }
    
    //

    viewer.data().set_mesh(float2double(V), F);
    viewer.data().set_face_based(true);

    viewer.core().camera_eye = Eigen::Vector3f(0.f, 0.f, 25.f);

    viewer.launch_init();
    viewer.draw();

    // Allocate temporary buffers
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> G(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> B(1280, 800);
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> A(1280, 800);

    // Draw the scene in the buffers
    viewer.core().draw_buffer(viewer.data(), false, R, G, B, A);

    // Save it to a PNG
    string outpng = t_folder + "\\Image" + padseq(seq) + ".png";

    cout << outpng << endl;
    
    //missing header file.
    igl::png::writePNG(R, G, B, A, outpng);
    //
    viewer.launch_rendering(false);
    viewer.launch_shut();


}

void GraphicsEngine::reset()
{
    DynamicVec.clear();
    StaticVec.clear();
}


void GraphicsEngine::run(string project_folder)
{
    string image_folder = project_folder + "images";
    _mkdir(image_folder.c_str());
    short sequence = 0;
    for (const auto& entry : fs::directory_iterator(project_folder)) {
        string scenefolder = entry.path().string();
        cout << "loading scene: " << scenefolder << endl;
        load_scene(scenefolder);
        save_scene(image_folder, sequence);
        reset();
        sequence++;
    }

}


