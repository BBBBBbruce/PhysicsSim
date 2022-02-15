#include "StressFEM.h"
#include<Eigen/Dense>

using namespace std;
using namespace Eigen;



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

StressFEM::StressFEM()
{
}

StressFEM::StressFEM(string t_path)
{
    vector<StaticObj>StaticVec;
    vector<DynamicObj>DynamicVec;
    vector<Eigen::MatrixXd>TransVelVec;
    this->tpath = t_path;
}

vector<StaticObj> StressFEM::getStaticObjs()
{
    return StaticVec;
}

vector<DynamicObj> StressFEM::getDynamicObjs()
{
    return DynamicVec;
}

void StressFEM::load_scene(int pre_seq)
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
            Eigen::MatrixXd P;
            Eigen::MatrixXi Tri, Tet;
            Eigen::VectorXi TriTag, TetTag;
            std::vector<std::string> XFields, EFields;
            std::vector<Eigen::MatrixXd> XF, TriF, TetF;
            igl::readMSH(it.value()["position_path"], P, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);

            DynamicObj dtmp(
                it.key(),
                P, Tet, Tri, TriTag, TetTag, XFields, EFields,
                XF, TriF, TetF,
                Eigen::Vector3d(),
                Eigen::Vector3d(it.value()["angular_velocity"][0], it.value()["angular_velocity"][1], it.value()["angular_velocity"][2]),
                it.value()["mass"]
            );
            DynamicVec.push_back(dtmp);
            MatrixXd vel = XfFromcsv(string(it.value()["trans_vel_path"]).c_str());
            TransVelVec.push_back(vel); // stable(same order as DynamicVec)
            MatrixXd x = XfFromcsv(string(it.value()["rigid_position_path"]).c_str());
            RigidPosVec.push_back(x);
            double damp = 2 * sqrt(it.value()["mass"] / P.rows())*zeta; // *w
            DampVec.push_back(damp);

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
                X, Tet, Tri, TriTag, TetTag, XFields, EFields,
                XF, TriF, TetF, it.value()["position_path"]
            );
            StaticVec.push_back(stmp);
        }
    }
}

void StressFEM::save_scene(int seq)
{
    json jout;

    for (auto i = 0; i < DynamicVec.size(); i++) {
        string path_p = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_p.msh";
        DynamicVec[i].writemsh(path_p);
        string path_v = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_v.csv";
        Xf2csv(path_v.c_str(), TransVelVec[i]);
        string path_x = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_x.csv";
        Xf2csv(path_x.c_str(), RigidPosVec[i]);
        //debug only
        string path_pp = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_p.csv";
        Xf2csv(path_pp.c_str(), DynamicVec[i].get_position());

        Eigen::Vector3d lv = DynamicVec[i].get_linear_velocity();
        Eigen::Vector3d av = DynamicVec[i].get_angular_velocity();

        json jtmp;
        jtmp["rigid_position_path"] = path_x;
        jtmp["position_path"] = path_p;
        jtmp["trans_vel_path"] = path_v;
        jtmp["angular_velocity"] = { av[0],av[1],av[2] };
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

void StressFEM::InitConfigs(string targetpath, json currentconfig)
{
    json jout;

    make_dir_win(targetpath, 0);
    Eigen::MatrixXd X;
    Eigen::MatrixXd V;
    Eigen::MatrixXi Tri, Tet;
    Eigen::VectorXi TriTag, TetTag;
    std::vector<std::string> XFields, EFields;
    std::vector<Eigen::MatrixXd> XF, TriF, TetF;
 

    for (auto it = currentconfig.begin(); it != currentconfig.end(); ++it)
    {
        if (it.key() == "physics") {
            set_physics_params(it.value()["damping_coef"], it.value()["restitution"], it.value()["collision_time"], it.value()["friction"], it.value()["Young_s_modulus"], it.value()["Poisson_Ratio"]);
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
            
            Eigen::Vector3d trans_vel = Eigen::Vector3d(it.value()["translation_velocity"][0], it.value()["translation_velocity"][1], it.value()["translation_velocity"][2]);
            Eigen::MatrixXd TransVel = trans_vel.replicate(1,X.rows()).transpose();
            
            string path_x = targetpath + "0000" + "/" + it.key() + "_x.csv";
            string path_v = targetpath + "0000" + "/" + it.key() + "_v.csv";
            Xd2csv(path_v.c_str(), TransVel);
            Xd2csv(path_x.c_str(), _X);

            // TODO save matrixxf only

            json jtmp;
            jtmp["rigid_position_path"] = path_x;
            jtmp["position_path"] = path_p;
            jtmp["trans_vel_path"] = path_v;
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

void StressFEM::reset()
{
    StaticVec.clear();
    DynamicVec.clear();
    TransVelVec.clear();
    RigidPosVec.clear();
    DampVec.clear();

}

void StressFEM::set_physics_params(double zeta, double restitution, double collision_t, double friction, double young, double poisson)
{
    zeta = zeta;
    e = restitution;
    tc = collision_t;
    u = friction;
    Young = young;
    Poisson = poisson;

    YModulus = MatrixXd(6, 6);
    YModulus << 1 - Poisson, Poisson    , Poisson    , 0              , 0              , 0,
                Poisson    , 1 - Poisson, Poisson    , 0              , 0              , 0,
                Poisson    , Poisson    , 1 - Poisson, 0              , 0              , 0,
                0          , 0          , 0          , 1 - 2 * Poisson, 0              , 0,
                0          , 0          , 0          , 0              , 1 - 2 * Poisson, 0,
                0          , 0          , 0          , 0              , 0              , 1 - 2 * Poisson;
    YModulus *= Young / (1 + Poisson) / (1 - 2 * Poisson);
}

MatrixXd HookeLaw( MatrixXd epsilon, Matrix<double, 6, 6 > modulus) {
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

void StressFEM::run(float delta_t, int seq)
{
    // Implement the physics
    // linearly iterate through all tetrahedrons
    // assume no friction yet
    for (auto i = 0; i < DynamicVec.size(); i++) {
        MatrixXd p_s = DynamicVec[i].get_position();
        MatrixXi tet = DynamicVec[i].get_tetrahedrons();
        MatrixXd v = TransVelVec[i];
        double m = DynamicVec[i].get_mass();
        Vector3d cm = DynamicVec[i].get_cm();
        MatrixXd x = RigidPosVec[i];
        //cout << x << endl;
        vector<MatrixXd> _x;
        for (auto j = 0; j < tet.rows(); j++) {
            MatrixXd x0 = x.row(tet(j, 0));
            MatrixXd tmp(3, 3); 
            tmp << x.row(tet(j, 1)) - x0, x.row(tet(j, 2)) - x0, x.row(tet(j, 3)) - x0;
            //MatrixXd TMP_X = x.row(tet(j, 1)) - x0;
            //float norm = TMP_X.norm();

            _x.push_back(tmp.inverse());
            //cout << tmp << endl;
        }
        
        MatrixXd G = m / x.rows() * gravity.transpose().replicate(x.rows(), 1);
        MatrixXd f_total;
        auto [collide, contact_points] = CD_table_FEM(p_s);
        MatrixXd f_internal = MatrixXd::Zero(x.rows(), x.cols());
        Matrix<double, 1, 3> ff1;
        Matrix<double, 1, 3> ff2;
        Matrix<double, 1, 3> ff3;
        Matrix<double, 1, 3> ff4;
        for (auto j = 0; j < tet.rows(); j++) {

            Matrix<double, 1, 3> p0 = p_s.row(tet(j, 1)) - p_s.row(tet(j, 0));
            Matrix<double, 1, 3> p1 = p_s.row(tet(j, 2)) - p_s.row(tet(j, 0));
            Matrix<double, 1, 3> p2 = p_s.row(tet(j, 3)) - p_s.row(tet(j, 0));
            Matrix<double, 1, 3> p3 = p_s.row(tet(j, 2)) - p_s.row(tet(j, 1));
            Matrix<double, 1, 3> p4 = p_s.row(tet(j, 3)) - p_s.row(tet(j, 1));

            MatrixXd p_c(3, 3);
            p_c << p0, p1, p2;
            p_c = p_c * _x[j];

            MatrixXd delta_u = p_c - Matrix3d::Identity();
            //cout << delta_u << endl;
            MatrixXd strain = 0.5 * (delta_u + delta_u.transpose() + delta_u.transpose() * delta_u);
            //MatrixXd strain = p_c.transpose()*p_c - Matrix3d::Identity();
            cout << "strain: " << strain << endl;
            //cout << strain << endl;
            MatrixXd stress = HookeLaw(strain, YModulus); 
            cout << "stress: " << stress << endl;
            //stress /= 10;
            //Matrix<float, 1, 3> ff1 = p0.cross(p1) * signbit(p0.cross(p1).dot(p4)) * stress * 0.5;
            ff1 = p0.cross(p1) * stress * 0.5;
            ff1 *= signbit(p0.cross(p1).dot(p4)) == 1 ? -1 : 1;
            ff2 = p0.cross(p2) * stress * 0.5;
            ff2 *= signbit(p0.cross(p2).dot(p3)) == 1 ? -1 : 1;
            ff3 = p1.cross(p2) * stress * 0.5;
            ff3 *= signbit(p1.cross(p2).dot(p0)) == 1 ? -1 : 1;
            ff4 = p3.cross(p4)  * stress * 0.5;
            ff4 *= signbit(p3.cross(p4).dot(-p0)) == 1 ? -1 : 1;
            //ff1 *= -1;
            //ff2 *= -1;
            //ff3 *= -1;
            //ff4 *= -1;
            f_internal.row(tet(j, 0)) += (ff1 + ff2 + ff3) / 3;
            f_internal.row(tet(j, 1)) += (ff1 + ff2 + ff4) / 3;
            f_internal.row(tet(j, 2)) += (ff1 + ff4 + ff3) / 3;
            f_internal.row(tet(j, 3)) += (ff4 + ff2 + ff3) / 3;
        }

        MatrixXd f_damping = -v * zeta; 
        if (!collide)
            f_total = G + f_internal +f_damping;
        else {
            MatrixXd v_stress = MatrixXd::Zero(f_internal.rows(), f_internal.cols());
            v_stress.col(1) = f_internal.col(1);
            MatrixXd f_collide = -(1 + e) * v * m / x.rows() / tc - G - v_stress;
            MatrixXd zero_buff = MatrixXd::Zero(f_collide.rows(), f_collide.cols());

            for (auto i = 0; i < contact_points.size(); i++) {
                zero_buff.row(contact_points[i]) = f_collide.row(contact_points[i]);
            }
            f_total = G + zero_buff + f_internal +f_damping;
            //cout << f_total << endl;
        }
        //f_total += f_internal;
        


        //================ debugging ==========================
        string path_f_internal = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_internal.csv";
        Xf2csv(path_f_internal.c_str(), f_internal);
        string path_f_total = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_total.csv";
        Xf2csv(path_f_total.c_str(), f_total);


        string path_ff1 = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_ff1.csv";
        Xf2csv(path_ff1.c_str(), ff1);
        string path_ff2 = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_ff2.csv";
        Xf2csv(path_ff2.c_str(), ff2);
        string path_ff3 = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_ff3.csv";
        Xf2csv(path_ff3.c_str(), ff3);
        string path_ff4 = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_ff4.csv";
        Xf2csv(path_ff4.c_str(), ff4);
  
        //================ debugging ==========================

        MatrixXd vf = v + f_total * delta_t / m * x.rows();
        //MatrixXd delta_x = delta_t * (v + vf) / 2;
        MatrixXd delta_x = delta_t * vf / 2;
        p_s += delta_x;
        //p_s += apply_poisson(delta_x,Poisson);
        DynamicVec[i].update_state(p_s, Vector3d(), Vector3d()); // no w 
        TransVelVec[i] = vf;
        RigidPosVec[i] = x;
    }
}

