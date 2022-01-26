#include "NewtonRigid.h"
#include "Collision.h"
using namespace std;
using namespace Eigen;
//glm::vec3 gravity(0.0, 0.0, -9.8);


Eigen::Vector3f gravity(0.0, -10.0, 0.0);

NewtonRigid::NewtonRigid()
{
    
}

NewtonRigid::NewtonRigid(string t_path)
{
    //cout << targetpath << endl;
    vector<StaticObj>StaticVec;
    vector<DynamicObj>DynamicVec;
    this->tpath = t_path;
    //cout <<"engine: "<< targetpath << endl;
}


bool NewtonRigid::collision_detection()
{
    return false;
}

/*
void NewtonRigid::ParseWorld(json objectlist)
{

    //cout << "bbb" << endl;
    
    for (auto it = objectlist.begin(); it != objectlist.end(); ++it)
    {
        //cout << "aaa" << endl;
        
        if (it.value()["type"] == "dynamic") {
            //cout << it.value()["position"].get<> << endl;
            DynamicObj dtmp(
                it.key(),
                it.value()["path"],
                Eigen::Vector3f({ it.value()["position"][0],it.value()["position"][1], it.value()["position"][2] }),
                Eigen::Vector3f({ it.value()["scale"][0],it.value()["scale"][1], it.value()["scale"][2] }),
                Eigen::Vector3f({ it.value()["rotation"][0],it.value()["rotation"][1], it.value()["rotation"][2] }),
                Eigen::Vector3f({ it.value()["velocity"][0],it.value()["velocity"][1], it.value()["velocity"][2] }),
                it.value()["mass"]
            );
            DynamicVec.push_back(dtmp);

        }
        else if (it.value()["type"] == "static") {

            StaticObj stmp(
                it.key(),
                it.value()["path"],
                Eigen::Vector3f({ it.value()["position"][0],it.value()["position"][1], it.value()["position"][2] }),
                Eigen::Vector3f({ it.value()["scale"][0],it.value()["scale"][1], it.value()["scale"][2] }),
                Eigen::Vector3f({ it.value()["rotation"][0],it.value()["rotation"][1], it.value()["rotation"][2] })
                
            );
            StaticVec.push_back(stmp);

        }
    }
}

*/

vector<StaticObj> NewtonRigid::getStaticObjs()
{
    return StaticVec;
}

vector<DynamicObj> NewtonRigid::getDynamicObjs()
{
    return DynamicVec;
}


void NewtonRigid::load_scene(int pre_seq)
{   

    string scene_in = tpath + padseq(pre_seq) + "/Scene.json";
    std::ifstream ifs(scene_in);
    json j;

    try
    {
        j = json::parse(ifs);
    }
    catch (json::parse_error& ex)
    {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
    }

    for (auto it = j["Objects"].begin(); it != j["Objects"].end(); ++it) {
        if (it.value()["type"] == "dynamic") {
            Eigen::MatrixXd X;
            Eigen::MatrixXi Tri, Tet;
            Eigen::VectorXi TriTag, TetTag;
            std::vector<std::string> XFields, EFields;
            std::vector<Eigen::MatrixXd> XF, TriF, TetF;

            igl::readMSH(it.value()["position_path"], X, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
            
            DynamicObj dtmp(
                it.key(),
                double2float(X), Tet, Tri, TriTag, TetTag, XFields, EFields,
                cast2float(XF), cast2float(TriF), cast2float(TetF),
                Eigen::Vector3f(it.value()["linear_velocity"][0], it.value()["linear_velocity"][1], it.value()["linear_velocity"][2]),
                Eigen::Vector3f(it.value()["mass_centre"][0], it.value()["mass_centre"][1], it.value()["mass_centre"][2]),
                Eigen::Vector3f(it.value()["angular_velocity"][0], it.value()["angular_velocity"][1], it.value()["angular_velocity"][2]),
                it.value()["mass"]
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



bool collision_check(MatrixXf vertices, MatrixXi tets) {


    for (auto i = 0; i < tets.rows(); i++) {
        //inline double vertex1 = vertices(tets(i, 0), 1);
        //inline double vertex2 = vertices(tets(i, 1), 1);
        //inline double vertex3 = vertices(tets(i, 2), 1);
        //inline double vertex4 = vertices(tets(i, 3), 1);


        if (vertices(tets(i, 0), 1) <= 0.0) {
            return true;
        }
        else if (vertices(tets(i, 1), 1) <= 0.0) {
            return true;
        }
        else if (vertices(tets(i, 2), 1) <= 0.0) {
            return true;
        }
        else if (vertices(tets(i, 3), 1) <= 0.0) {
            return true;
        }
    }
    return false;
}


//+++++++++++++++++++++++++++++++++++++++++++


Eigen::Matrix4f rotate_eigen_api(Vector3f theta) {
    Affine3f rx(AngleAxisf(theta[0], Vector3f::UnitX()));
    Matrix4f rotationx = rx.matrix();
    Affine3f ry(AngleAxisf(theta[1], Vector3f::UnitY()));
    Matrix4f rotationy = ry.matrix();
    Affine3f rz(AngleAxisf(theta[2], Vector3f::UnitZ()));
    Matrix4f rotationz = rz.matrix();
    return rotationx * rotationy * rotationz;
}

void rotate(MatrixXf &x, Vector3f&theta, Vector3f &cm) {
    // define theta: 
    // Right hand rule: (+) --> counter-clockwise, (-) --> clockwise
    // x rotates about Vector3f::UnitX(), y rotates about Vector3f::UnitY(), z rotates about Vector3f::UnitZ()
    Affine3f tl(Translation3f(-cm[0], -cm[1], -cm[2]));
    Matrix4f translation = tl.matrix();
    Affine3f tlb(Translation3f(cm[0], cm[1], cm[2]));
    Matrix4f translation_back = tlb.matrix();

    MatrixXf pos_tmp = x;
    pos_tmp.conservativeResize(pos_tmp.rows(), 4);// Make V n*4 matrix
    pos_tmp.col(3).setOnes();
    pos_tmp = (translation_back * rotate_eigen_api(theta) * translation * pos_tmp.transpose()).transpose();
    x = MatrixXf(pos_tmp.rows(), 3);
    x.col(0) = pos_tmp.col(0);
    x.col(1) = pos_tmp.col(1);
    x.col(2) = pos_tmp.col(2);
}

Eigen::Vector3f rowwise_projection(Eigen::Vector3f vec, Eigen::Vector3f dir_vec) {
    return dir_vec.normalized() * vec.dot(dir_vec);
}

Eigen::MatrixXf projection(Eigen::MatrixXf matrix, Eigen::Vector3f dir_vec) {
    auto dots = matrix * dir_vec;
    MatrixXf output = dir_vec.normalized().replicate(matrix.rows(), 1);

    output = output.array() * dots.replicate(1,3).array();// debug til runtime
    return output;
}

Eigen::Vector3f project_cos(Eigen::Vector3f vec, Eigen::Vector3f dir_vec) {
    auto dots = vec.transpose() * dir_vec;
    Vector3f output = dir_vec.normalized() * dots;
    return output;
}

Eigen::Vector3f project_sin(Eigen::Vector3f vec, Eigen::Vector3f dir_vec) {
    auto dots = vec.transpose() * dir_vec;
    Vector3f output = dir_vec.normalized() * dots;
    return vec - output;
}



void NewtonRigid::run(float delta_t,int seq)
{   
 
    float r = 1; 
    for (auto i = 0; i < DynamicVec.size(); i++) {

        // pre-defined parameters
        MatrixXf xf;
        Vector3f vf;
        Vector3f wf;
        Vector3f cmf;

        float mass = DynamicVec[i].get_mass();
        MatrixXf x0 = DynamicVec[i].get_position();
        Vector3f v0 = DynamicVec[i].get_linear_velocity();
        Vector3f w0 = DynamicVec[i].get_angular_velocity();
        MatrixXi tets = DynamicVec[i].get_tetrahedrons();
        Vector3f cm = DynamicVec[i].get_cm();

        MatrixXf x = x0.rowwise() + (v0 * delta_t + 0.5 * gravity * delta_t * delta_t).transpose();
        //bool collide = collision_check(x, tets);
        //auto [collide, contact_p] = CD_bowl(x);
        //auto [collide, contact_p] = CD_table(x);
        auto [collide, contact_p] = CD_bowl_wide(x);
        //collide = false;

        cmf = cm + v0 * delta_t + 0.5 * gravity * delta_t * delta_t;
        Vector3f theta = w0 * delta_t;
        rotate(x, theta, cmf);
        
        if (!collide) {
            vf = v0 + gravity * delta_t;
            wf = w0;

        }
        else {
            //std::cout << endl;
            contact_p = contact_p - (v0 * delta_t + 0.5 * gravity * delta_t * delta_t);
            Vector3f r_dir = (contact_p - cm).normalized();

            Vector3f vc = project_cos(v0, r_dir);
            Vector3f vs = v0 - vc;
            Vector3f gc = project_cos(gravity, r_dir);
            Vector3f gs = gravity - gc;

            Vector3f N_impact = mass  * (-e - 1) * vc /tc + mass * gc;// simplification of vc
            Vector3f fs = u * N_impact.norm() * (vs + w0.cross(r_dir)).normalized();

            Vector3f vf1 = -vc * (1 + e) + gc * (delta_t - tc) ; 

            Vector3f vf2 = delta_t * gs + fs*tc/mass ;
            vf = vf1 + vf2+v0;

            float I = 0.4 * mass * r * r;
            //Vector3f fs = u * mass / tc * (-e - 1) * vc;
            wf = fs.cross(r_dir) * tc / I + w0;

        }

        DynamicVec[i].update_state(x, vf, wf, cmf);
    }

}

void NewtonRigid::set_physics_params(float restitution, float collision_t, float friction)
{
    e = restitution;
    tc = collision_t;
    u = friction;
}


//++++++++++++++++++++
void NewtonRigid::save_scene(int seq)
{   
    json jout;

    for (auto i = 0; i < DynamicVec.size(); i++) {
        string path_p = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_p.msh";
        DynamicVec[i].writemsh(path_p);

        Eigen::Vector3f lv = DynamicVec[i].get_linear_velocity();
        Eigen::Vector3f av = DynamicVec[i].get_angular_velocity();
        Eigen::Vector3f cm = DynamicVec[i].get_cm();

        json jtmp;
        jtmp["position_path"] = path_p;
        jtmp["linear_velocity"] = {lv[0],lv[1],lv[2]};
        jtmp["angular_velocity"] = {av[0],av[1],av[2]};
        jtmp["mass_centre"] = {cm[0],cm[1],cm[2]};
        jtmp["mass"] = DynamicVec[i].get_mass();
        jtmp["type"] = "dynamic";
        jout["Objects"][DynamicVec[i].name] = jtmp;
    }
    for (auto i = 0; i < StaticVec.size(); i++) {
        json jtmp;
        jtmp["position_path"] = StaticVec[i].get_loadingpath();
        jtmp["type"] = "static";
        jout["Objects"][StaticVec[i].name] = jtmp;
    }

    string path_scene = tpath + padseq(seq) + "/" + "Scene.json";
    std::ofstream o(path_scene);
    o << std::setw(4) << jout << std::endl;

}

//#include<cgal/IO/File_tetgen.h>
void NewtonRigid::create_aabb_tree() {

}

void NewtonRigid::reset()
{
    DynamicVec.clear();
    StaticVec.clear();
}


