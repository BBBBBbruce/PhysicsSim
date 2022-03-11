#include "NewtonRigid.h"
#include "Collision.h"
#include "utiles.h"

using namespace std;
using namespace Eigen;

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

vector<StaticObj*> NewtonRigid::getStaticObjs()
{
    return StaticVec;
}

vector<DynamicObj*> NewtonRigid::getDynamicObjs()
{
    return DynamicVec;
}

void NewtonRigid::InitConfigs(string targetpath, json currentconfig, float tc)
{
    json jout;

    make_dir_win(targetpath, 0);
    Eigen::MatrixXd X;
    Eigen::MatrixXi Tri, Tet;
    Eigen::VectorXi TriTag, TetTag;
    std::vector<std::string> XFields, EFields;
    std::vector<Eigen::MatrixXd> XF, TriF, TetF;

    using namespace Eigen;

    for (auto it = currentconfig.begin(); it != currentconfig.end(); ++it)
    {
        if (it.key() == "physics") {
            set_physics_params(it.value()["restitution"], it.value()["collision_time"], it.value()["friction"]);
        }
        if (it.value()["type"] == "dynamic") {
            //cout << it.value()["position"].get<> << endl;

            igl::readMSH(it.value()["path"], X, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);

            X.conservativeResize(X.rows(), 4);// Make V n*4 matrix
            X.col(3).setOnes();

            Affine3d tl(Translation3d(it.value()["position"][0], it.value()["position"][1], it.value()["position"][2]));
            Matrix4d translation = tl.matrix();
            Affine3d sc(AlignedScaling3d(it.value()["scale"][0], it.value()["scale"][1], it.value()["scale"][2]));
            Matrix4d scale = sc.matrix();
            Affine3d rx(AngleAxisd(it.value()["rotation"][0], Vector3d::UnitX()));
            Matrix4d rotationx = rx.matrix();
            Affine3d ry(AngleAxisd(it.value()["rotation"][1], Vector3d::UnitY()));
            Matrix4d rotationy = ry.matrix();
            Affine3d rz(AngleAxisd(it.value()["rotation"][2], Vector3d::UnitZ()));
            Matrix4d rotationz = rz.matrix();

            //seems redundent, TODO: opt
            MatrixXd V_tmp = translation * rotationx * rotationy * rotationz * scale * X.transpose();
            X = V_tmp.transpose();
            MatrixXd _X(X.rows(), 3);
            _X.col(0) = X.col(0);
            _X.col(1) = X.col(1);
            _X.col(2) = X.col(2);
            string path_p = targetpath + "0000" + "/" + it.key() + "_p.msh";
            igl::writeMSH(path_p, _X, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);

            Eigen::Vector3d cm = Eigen::Vector3d(it.value()["mass_centre"][0], it.value()["mass_centre"][1], it.value()["mass_centre"][2]);
            Eigen::Vector4d cm_tmp = Vector4d(cm[0], cm[1], cm[2], 1.0);
            cm_tmp = translation * rotationx * rotationy * rotationz * scale * cm_tmp;
            //cm = (translation* rotationx* rotationy* rotationz* scale* cm_tmp).head<3>();
            // TODO adding initial velocity with .msh file of a complete verision of velocity)

            json jtmp;
            jtmp["position_path"] = path_p;
            jtmp["linear_velocity"] = it.value()["linear_velocity"];
            jtmp["angular_velocity"] = it.value()["angular_velocity"];
            jtmp["mass"] = it.value()["mass"];
            jtmp["type"] = "dynamic";
            jout["Objects"][it.key()] = jtmp;

        }


        else if (it.value()["type"] == "static") {

            igl::readMSH(it.value()["path"], X, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);

            X.conservativeResize(X.rows(), 4);// Make V n*4 matrix
            X.col(3).setOnes();

            Affine3d tl(Translation3d(it.value()["position"][0], it.value()["position"][1], it.value()["position"][2]));
            Matrix4d translation = tl.matrix();
            Affine3d sc(AlignedScaling3d(it.value()["scale"][0], it.value()["scale"][1], it.value()["scale"][2]));
            Matrix4d scale = sc.matrix();
            Affine3d rx(AngleAxisd(it.value()["rotation"][0], Vector3d::UnitX()));
            Matrix4d rotationx = rx.matrix();
            Affine3d ry(AngleAxisd(it.value()["rotation"][1], Vector3d::UnitY()));
            Matrix4d rotationy = ry.matrix();
            Affine3d rz(AngleAxisd(it.value()["rotation"][2], Vector3d::UnitZ()));
            Matrix4d rotationz = rz.matrix();

            //seems redundent, TODO: opt
            MatrixXd V_tmp = translation * rotationx * rotationy * rotationz * scale * X.transpose();
            X = V_tmp.transpose();
            MatrixXd _X(X.rows(), 3);
            _X.col(0) = X.col(0);
            _X.col(1) = X.col(1);
            _X.col(2) = X.col(2);
            string path_p = targetpath + "0000" + "/" + it.key() + "_s.msh";
            igl::writeMSH(path_p, _X, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
            json jtmp;
            jtmp["position_path"] = path_p;
            jtmp["type"] = "static";
            jout["Objects"][it.key()] = jtmp;

        }
    }


    string path_scene = targetpath + "0000" + "/" + "Scene.json";
    std::ofstream o(path_scene);
    o << std::setw(4) << jout << std::endl;
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

            DynamicObj* dtmp = new DynamicObj(
                it.key(),
                X, Tet, Tri, TriTag, TetTag, XFields, EFields,
                XF, TriF, TetF,
                Eigen::Vector3d(it.value()["linear_velocity"][0], it.value()["linear_velocity"][1], it.value()["linear_velocity"][2]),
                Eigen::Vector3d(it.value()["angular_velocity"][0], it.value()["angular_velocity"][1], it.value()["angular_velocity"][2]),
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
            StaticObj* stmp = new StaticObj(
                it.key(),
                X, Tet, Tri, TriTag, TetTag, XFields, EFields,
                XF, TriF, TetF, it.value()["position_path"]
            );
            StaticVec.push_back(stmp);
        }
    }
}

void NewtonRigid::save_scene(int seq)
{
    json jout;

    for (auto i = 0; i < DynamicVec.size(); i++) {
        string path_p = tpath + padseq(seq) + "/" + DynamicVec[i]->name + "_p.msh";
        DynamicVec[i]->writemsh(path_p);

        Eigen::Vector3d lv = DynamicVec[i]->get_linear_velocity();
        Eigen::Vector3d av = DynamicVec[i]->get_angular_velocity();
        Eigen::Vector3d cm = DynamicVec[i]->get_cm();

        json jtmp;
        jtmp["position_path"] = path_p;
        jtmp["linear_velocity"] = { lv[0],lv[1],lv[2] };
        jtmp["angular_velocity"] = { av[0],av[1],av[2] };
        jtmp["mass"] = DynamicVec[i]->get_mass();
        jtmp["type"] = "dynamic";
        jout["Objects"][DynamicVec[i]->name] = jtmp;
    }
    for (auto i = 0; i < StaticVec.size(); i++) {
        json jtmp;
        jtmp["position_path"] = StaticVec[i]->get_loadingpath();
        jtmp["type"] = "static";
        jout["Objects"][StaticVec[i]->name] = jtmp;
    }

    string path_scene = tpath + padseq(seq) + "/" + "Scene.json";
    std::ofstream o(path_scene);
    o << std::setw(4) << jout << std::endl;

}

bool collision_check(MatrixXd vertices, MatrixXi tets) {


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

void NewtonRigid::run(float delta_t, int seq)
{

    double r = 1;
    for (auto i = 0; i < DynamicVec.size(); i++) {

        // pre-defined parameters
        MatrixXd xf;
        Vector3d vf;
        Vector3d wf;
        Vector3d cmf;

        double mass = DynamicVec[i]->get_mass();
        MatrixXd x0 = DynamicVec[i]->get_position();
        Vector3d v0 = DynamicVec[i]->get_linear_velocity();
        Vector3d w0 = DynamicVec[i]->get_angular_velocity();
        MatrixXi tets = DynamicVec[i]->get_tetrahedrons();
        Vector3d cm = DynamicVec[i]->get_cm();

        MatrixXd x = x0.rowwise() + (v0 * delta_t + 0.5 * gravity * delta_t * delta_t).transpose();
        //bool collide = collision_check(x, tets);
        //auto [collide, contact_p] = CD_bowl(x);
        //auto [collide, contact_p] = CD_table(x);
        auto [collide, contact_p] = CD_bowl_wide(x);
        //collide = false;

        cmf = cm + v0 * delta_t + 0.5 * gravity * delta_t * delta_t;
        Vector3d theta = w0 * delta_t;
        rotate(x, theta, cmf);

        if (!collide) {
            vf = v0 + gravity * delta_t;
            wf = w0;

        }
        else {
            //std::cout << endl;
            contact_p = contact_p - (v0 * delta_t + 0.5 * gravity * delta_t * delta_t);
            Vector3d r_dir = (contact_p - cm).normalized();

            Vector3d vc = project_cos(v0, r_dir);
            Vector3d vs = v0 - vc;
            Vector3d gc = project_cos(gravity, r_dir);
            Vector3d gs = gravity - gc;

            Vector3d N_impact = mass * (-e - 1) * vc / tc + mass * gc;// simplification of vc
            Vector3d fs = u * N_impact.norm() * (vs + w0.cross(r_dir)).normalized();

            Vector3d vf1 = -vc * (1 + e) + gc * (delta_t - tc);

            Vector3d vf2 = delta_t * gs + fs * tc / mass;
            vf = vf1 + vf2 + v0;

            double I = 0.4 * mass * r * r;
            //Vector3d fs = u * mass / tc * (-e - 1) * vc;
            wf = fs.cross(r_dir) * tc / I + w0;

        }

        DynamicVec[i]->update_state(x, vf, wf);
    }

}


