#include "StressFEM.h"
#include<Eigen/Dense>
#include "igl/volume.h"

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
    vector<Eigen::MatrixXd>currentVec;
    vector<Eigen::MatrixXd>lastVec;
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

void StressFEM::InitConfigs(string targetpath, json currentconfig, float tc)
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
    }

    for (auto it = currentconfig.begin(); it != currentconfig.end(); ++it)
    {
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
            Eigen::MatrixXd last_vel = trans_vel.replicate(1, X.rows()).transpose();
            Eigen::MatrixXd current_vel;

            // ================== init rendering ======================
            auto [collide, contact_points] = CD_table_FEM(X);
            double rho = it.value()["mass"];
            Eigen::VectorXd vol;
            igl::volume(_X, Tet, vol);
            double volum = abs(vol.sum());
            double m = volum * rho;
            cout << "volume: " << volum << endl;
            cout << "mass: " << m << endl;
            cout << "density: " << rho << endl;
            MatrixXd G = m * gravity.transpose().replicate(X.rows(), 1);
            MatrixXd f_external;
            if (!collide)
                //f_total = G + f_internal; //+f_damping;
                f_external = G;
            else {
                MatrixXd f_collide = -(1 + e) * last_vel * m / tc - G;
                //cout << "f_collide:" << endl << f_collide << endl;
                MatrixXd zero_buff = MatrixXd::Zero(f_collide.rows(), f_collide.cols());

                for (auto i = 0; i < contact_points.size(); i++) {
                    zero_buff.row(contact_points[i]) = f_collide.row(contact_points[i]);
                }
                //f_total = G + zero_buff + f_internal +f_damping;
                f_external = G + zero_buff;
                //cout << f_total << endl;
            }
            //cout << f_external << endl;
            current_vel = f_external * tc / m;
            // =============== end of init rendering ====================

            string path_x = targetpath + "0000" + "/" + it.key() + "_x.csv";
            string path_vl = targetpath + "0000" + "/" + it.key() + "_vl.csv";
            string path_vc = targetpath + "0000" + "/" + it.key() + "_vc.csv";
            Xd2csv(path_vl.c_str(), last_vel);
            Xd2csv(path_vc.c_str(), current_vel);
            Xd2csv(path_x.c_str(), _X);

            // TODO save matrixxf only

            json jtmp;
            jtmp["rigid_position_path"] = path_x;
            jtmp["position_path"] = path_p;
            jtmp["last_velocity"] = path_vl;
            jtmp["current_velocity"] = path_vc;
            jtmp["angular_velocity"] = it.value()["angular_velocity"];
            jtmp["mass"] = m;
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
            MatrixXd vl = XfFromcsv(string(it.value()["last_velocity"]).c_str());
            lastVec.push_back(vl); // stable(same order as DynamicVec)
            MatrixXd vc = XfFromcsv(string(it.value()["current_velocity"]).c_str());
            currentVec.push_back(vc); // stable(same order as DynamicVec)
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
        string path_vl = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_vl.csv";
        Xf2csv(path_vl.c_str(), lastVec[i]);
        string path_vc = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_vc.csv";
        Xf2csv(path_vc.c_str(), currentVec[i]);
        string path_x = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_x.csv";
        Xf2csv(path_x.c_str(), RigidPosVec[i]);
        //debug only
        string path_pp = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_p.csv";
        Xf2csv(path_pp.c_str(), DynamicVec[i].get_position());

        //Eigen::Vector3d lv = DynamicVec[i].get_linear_velocity();
        //Eigen::Vector3d av = DynamicVec[i].get_angular_velocity();

        json jtmp;
        jtmp["rigid_position_path"] = path_x;
        jtmp["position_path"] = path_p;
        jtmp["last_velocity"] = path_vl;
        jtmp["current_velocity"] = path_vc;
        jtmp["angular_velocity"] = { 0.0,0.0,0.0};
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

void StressFEM::reset()
{
    StaticVec.clear();
    DynamicVec.clear();
    currentVec.clear();
    lastVec.clear();
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
  /*  YModulus << 1 - Poisson, Poisson    , Poisson    , 0              , 0              , 0,
                Poisson    , 1 - Poisson, Poisson    , 0              , 0              , 0,
                Poisson    , Poisson    , 1 - Poisson, 0              , 0              , 0,
                0          , 0          , 0          , 1 - 2 * Poisson, 0              , 0,
                0          , 0          , 0          , 0              , 1 - 2 * Poisson, 0,
                0          , 0          , 0          , 0              , 0              , 1 - 2 * Poisson;*/
    YModulus << 1 - Poisson, Poisson, Poisson, 0, 0, 0,
        Poisson, 1 - Poisson, Poisson, 0, 0, 0,
        Poisson, Poisson, 1 - Poisson, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0;
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

void TetrahedronElementVolume() {

}

void TetrahedronElementStiffness() {

}

void TetrahedronAssemble() {

}

void TetrahedronElementStresses() {

}

void TetrahedronElementPStresses() {

}



void StressFEM::run(float delta_t, int seq)
{
    // Implement the physics
    // linearly iterate through all tetrahedrons
    // assume no friction yet
    for (auto i = 0; i < DynamicVec.size(); i++) {
        MatrixXd pl = DynamicVec[i].get_position();
        MatrixXi tet = DynamicVec[i].get_tetrahedrons();
        MatrixXd vl = lastVec[i];
        MatrixXd vc = currentVec[i];
        double m = DynamicVec[i].get_mass();// not change data structure, test today
        Vector3d cm = DynamicVec[i].get_cm();
        MatrixXd x = RigidPosVec[i];
        

        //cout << "vl: " << endl << vl << endl;
        //cout << "vc: " << endl << vc << endl;
        //======================= new logic: ==========================
        MatrixXd pc = pl + (vl + vc) * 0.5 * delta_t;
        //+++++++++++++++++++++++++
        // TODO: self_collision check:
        //
        //+++++++++++++++++++++++++

        //cout << "pc: " << endl << pc << endl;
        vector<MatrixXd> _x;
        vector<MatrixXd> X0;
        for (auto j = 0; j < tet.rows(); j++) {
            MatrixXd x0 = x.row(tet(j, 0));
            MatrixXd tmp(3, 3);
            tmp << x.row(tet(j, 1)) - x0, x.row(tet(j, 2)) - x0, x.row(tet(j, 3)) - x0;
            _x.push_back(tmp.inverse());
            X0.push_back(tmp);
            //cout << "_x: " << endl << tmp.inverse() << endl;
            //cout << "X0: " << endl << tmp << endl;
        }
        
        MatrixXd fi = MatrixXd::Zero(x.rows(), x.cols());
        Matrix<double, 1, 3> ff1;
        Matrix<double, 1, 3> ff2;
        Matrix<double, 1, 3> ff3;
        Matrix<double, 1, 3> ff4;
        for (auto j = 0; j < tet.rows(); j++) {

            Matrix<double, 1, 3> p0 = pc.row(tet(j, 1)) - pc.row(tet(j, 0));
            Matrix<double, 1, 3> p1 = pc.row(tet(j, 2)) - pc.row(tet(j, 0));
            Matrix<double, 1, 3> p2 = pc.row(tet(j, 3)) - pc.row(tet(j, 0));
            Matrix<double, 1, 3> p3 = pc.row(tet(j, 2)) - pc.row(tet(j, 1));
            Matrix<double, 1, 3> p4 = pc.row(tet(j, 3)) - pc.row(tet(j, 1));

            MatrixXd p_c(3, 3);
            p_c << p0, p1, p2;
            MatrixXd delta_u = (p_c - X0[j]) * _x[j];
            MatrixXd strain = 0.5 * (delta_u + delta_u.transpose() + delta_u.transpose() * delta_u);
            //cout << "strain: " << endl << strain << endl;
            //cout << "strain: " << endl << strain << endl;
            //cout << strain << endl;
            MatrixXd stress = HookeLaw(strain, YModulus);
            //cout << "stress: " << endl << stress << endl;
            
            ff1 = p0.cross(p1) * stress * 0.5;
            ff1 *= signbit(p0.cross(p1).dot(p4)) == 1 ? -1 : 1;
            ff2 = p0.cross(p2) * stress * 0.5;
            ff2 *= signbit(p0.cross(p2).dot(p3)) == 1 ? -1 : 1;
            ff3 = p1.cross(p2) * stress * 0.5;
            ff3 *= signbit(p1.cross(p2).dot(p0)) == 1 ? -1 : 1;
            ff4 = p3.cross(p4) * stress * 0.5;
            ff4 *= signbit(p3.cross(p4).dot(-p0)) == 1 ? -1 : 1;

            fi.row(tet(j, 0)) += (ff1 + ff2 + ff3) / 3;
            fi.row(tet(j, 1)) += (ff1 + ff2 + ff4) / 3;
            fi.row(tet(j, 2)) += (ff1 + ff4 + ff3) / 3;
            fi.row(tet(j, 3)) += (ff4 + ff2 + ff3) / 3;
        }
        //cout << "fi: " << endl << fi << endl;
        MatrixXd G = m / x.rows() * gravity.transpose().replicate(x.rows(), 1);

        
        // return the 
        //float maxnorm = fi.rowwise().norm().maxCoeff();
        //cout << "fi: " << endl << fi << endl;
        if (fi.rowwise().norm().maxCoeff() > 2 * G.rowwise().norm().maxCoeff()) {
            //cout << "FLAG!!!FLAG!!!FLAG!!!FLAG!!!FLAG!!!FLAG!!!FLAG!!!FLAG!!!" << endl;
            fi = fi.normalized() * 1.6 * G.norm();
            MatrixXd fe;
            MatrixXd v_y = MatrixXd::Zero(vl.rows(), vl.cols());
            v_y.col(1) = vl.col(1);
            auto [collide, contact_points] = CD_table_FEM(pl);
            if (!collide) {
                //f_total = G + f_internal; //+f_damping;
                //cout << "not collide" << endl;
                fe = G;
            }
            else {
                //cout << "collide" << endl;
                MatrixXd v_stress = MatrixXd::Zero(fi.rows(), fi.cols());
                v_stress.col(1) = fi.col(1);
                MatrixXd f_collide = -(1 + e) * v_y * m / x.rows() / tc - G - v_stress;
                MatrixXd zero_buff = MatrixXd::Zero(f_collide.rows(), f_collide.cols());

                for (auto i = 0; i < contact_points.size(); i++) {
                    zero_buff.row(contact_points[i]) = f_collide.row(contact_points[i]);
                }
                //f_total = G + zero_buff + f_internal +f_damping;
                fe = G + zero_buff;
                //cout << f_total << endl;
            }

            MatrixXd f_total = fi + fe;
            vc = vl + f_total * delta_t / m * x.rows();
            //cout << "vc: " << endl << vc << endl;
            pc = pl + (vl + vc) * 0.5 * delta_t;
            //cout << "pc: " << endl << pc << endl;
            vc = -0.8*vc;
            //vc = -vc;
        }

        MatrixXd f_total;
        //cout << "pc: " << endl << pc << endl;
        MatrixXd fe;

        // update fi, pc, vc
        MatrixXd v_y = MatrixXd::Zero(vc.rows(), vc.cols());
        v_y.col(1) = vc.col(1);
        auto [collide, contact_points] = CD_table_FEM(pc);
        
        if (!collide) {
            //f_total = G + f_internal; //+f_damping;
            //cout << "not collide" << endl;
            fe = G;
        }
        else {
            //cout << "collide" << endl;
            MatrixXd v_stress = MatrixXd::Zero(fi.rows(), fi.cols());
            v_stress.col(1) = fi.col(1);
            MatrixXd f_collide = -(1 + e) * v_y * m / x.rows() / tc - G - v_stress;
            MatrixXd zero_buff = MatrixXd::Zero(f_collide.rows(), f_collide.cols());

            for (auto i = 0; i < contact_points.size(); i++) {
                zero_buff.row(contact_points[i]) = f_collide.row(contact_points[i]);
            }
            //f_total = G + zero_buff + f_internal +f_damping;
            fe = G + zero_buff;
            //cout << f_total << endl;
        }
        f_total = fi + fe;
       
        //cout << "fi: " << endl << fi << endl;
        //cout << "fe: " << endl << fe << endl;
        //cout << "fi.norm: " << endl << fi.norm() << endl;
        //cout << "fe.norm: " << endl << fe.norm() << endl;
        //cout << "f_total: " << endl << f_total << endl;
        //cout << "f_total.norm: " << endl << f_total.norm() << endl;
        //===================== end of new logic ======================

        //================ debugging ==========================
        string path_f_internal = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_internal.csv";
        Xf2csv(path_f_internal.c_str(), fi);
        string path_f_total = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_total.csv";
        Xf2csv(path_f_total.c_str(), f_total);


        //string path_ff1 = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_ff1.csv";
        //Xf2csv(path_ff1.c_str(), ff1);
        //string path_ff2 = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_ff2.csv";
        //Xf2csv(path_ff2.c_str(), ff2);
        //string path_ff3 = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_ff3.csv";
        //Xf2csv(path_ff3.c_str(), ff3);
        //string path_ff4 = tpath + padseq(seq) + "/" + DynamicVec[i].name + "_ff4.csv";
        //Xf2csv(path_ff4.c_str(), ff4);
  
        ////================ debugging ==========================

        //MatrixXd vf = v + f_total * delta_t / m * x.rows();
        ////MatrixXd delta_x = delta_t * (v + vf) / 2;
        //MatrixXd delta_x = delta_t * vf / 2;
        //p_s += delta_x;
        //p_s += apply_poisson(delta_x,Poisson);
        DynamicVec[i].update_state(pc, Vector3d(), Vector3d()); // no w 
        currentVec[i] = vc + f_total / m / x.rows() * delta_t;
        lastVec[i] = vc;
        RigidPosVec[i] = x;
    }
}

