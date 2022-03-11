#include "StressFEM.h"
#include "igl/volume.h"
#include "utiles.h"

using namespace std;
using namespace Eigen;

StressFEM::StressFEM()
{
}

StressFEM::StressFEM(string t_path)
{
    vector<StaticObj*>StaticVec;
    vector<DynamicObj*>DynamicVec;
    vector<Eigen::MatrixXd>currentVec;
    vector<Eigen::MatrixXd>lastVec;
    this->tpath = t_path;
}

vector<StaticObj*> StressFEM::getStaticObjs()
{
    return StaticVec;
}

vector<DynamicObj*> StressFEM::getDynamicObjs()
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
            set_physics_params( it.value()["restitution"], it.value()["collision_time"], it.value()["friction"]);
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
            string path_x = targetpath + "0000" + "/" + it.key() + "_x.csv";
            Xd2csv(path_x.c_str(), _X);
            

            //igl::writeMSH(path_p, _X, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);

            Eigen::Vector3d trans_vel = Eigen::Vector3d(it.value()["translation_velocity"][0], it.value()["translation_velocity"][1], it.value()["translation_velocity"][2]);
            Eigen::MatrixXd last_vel = trans_vel.replicate(1, X.rows()).transpose();
            //Eigen::MatrixXd current_vel;
            Eigen::MatrixXd current_vel=last_vel;
            // ================== init rendering ======================

            //MatrixXd G = m * gravity.transpose().replicate(X.rows(), 1);
            //MatrixXd f_external;

            //auto [collide, contact_points] = CD_table_FEM(X);
            //if (!collide)
            //    //f_total = G + f_internal; //+f_damping;
            //    f_external = G;
            //else {
            //    MatrixXd f_collide = -(1 + e) * last_vel * m / tc - G;
            //    //cout << "f_collide:" << endl << f_collide << endl;
            //    MatrixXd zero_buff = MatrixXd::Zero(f_collide.rows(), f_collide.cols());

            //    for (auto i = 0; i < contact_points.size(); i++) {
            //        zero_buff.row(contact_points[i]) = f_collide.row(contact_points[i]);
            //    }
            //    //f_total = G + zero_buff + f_internal +f_damping;
            //    f_external = G + zero_buff;
            //    //cout << f_total << endl;
            //}
            //cout << "position" << endl << _X << endl;
            //MatrixXd K;
            //int n = _X.rows();// number of vertices
            //K.resize(3 * n, 3 * n);
            //K.setIdentity();
            ////cout << "_X:" << endl << _X << endl;
            for (auto i = 0; i < Tet.rows(); i++) {
                MatrixXi tet_i = tet_re_order(Tet.row(i), _X.row(Tet(i, 0)), _X.row(Tet(i, 1)), _X.row(Tet(i, 2)), _X.row(Tet(i, 3)));
                Tet.row(i) = tet_i;
            }

            double rho = it.value()["mass"];//it is density
            Eigen::VectorXd vol;
            igl::volume(_X, Tet, vol);
            double volum = abs(vol.sum());
            double m = volum * rho;

            double m_tmp = m / Tet.rows() / 4;
            mass.resize(_X.rows(),1);
            mass.setZero();
            for (int i = 0; i < Tet.rows(); i++) {
                mass(Tet(i, 0)) += m_tmp;
                mass(Tet(i, 1)) += m_tmp;
                mass(Tet(i, 2)) += m_tmp;
                mass(Tet(i, 3)) += m_tmp;
            }


            //    Matrix<double, 4, 3> tet1;
            //    tet1 << _X.row(Tet(i, 0)), _X.row(Tet(i, 1)), _X.row(Tet(i, 2)), _X.row(Tet(i, 3));
            //    //cout << "tet1" << endl << tet1 << endl;
            //    //cout <<"YModulus: " << YModulus << endl;
            //    MatrixXd ki = TetrahedronElementStiffness(YModulus, tet1);
            //    cout << "k"<<i<<": \n" << ki.diagonal().transpose() << endl;
            //    cout << tet_i << endl;
            //    //cout << "ki_sigularity:" << endl << ki.inverse()*ki << endl;
            //    TetrahedronAssemble(K, ki, Tet.row(i));
            //    //cout << K << endl;
            //}
            //f_external.resize(f_external.rows() * 3, 1);
            //// =================== apply boundry condition =====================
            //// apply BC to K and fe
            //MatrixXd fe = f_external;
            ////cout << "fe" << fe << endl;

            ////MatrixXd k = K;
            //for (auto i = 0; i < contact_points.size(); i++) {
            //    removeRow(K, contact_points[i]*3+1);
            //    removeColumn(K, contact_points[i] * 3 + 1);//
            //    removeRow(fe, contact_points[i]*3+1);
            //}


            //// ================= finish apply boundry conditions ===============
            //
            ////cout << "shape: "  << "(" << fe.rows() << ", " << fe.cols() << ")" << endl;
            ////cout << "shape: " << "(" << k.rows() << ", " << k.cols() << ")" << endl;
            //cout << "K:" << endl << K.diagonal() << endl;
            ////cout << "K.inverse():" << endl << K.inverse() << endl;
            ////out << "K-K.transpose()" << endl << K - K.transpose() << endl;
            ////cout << "f_external:" << endl << f_external << endl;
            //
            //MatrixXd U = K.inverse() * fe;
            ////cout << "U:" << endl << U << endl;
            //for (auto i = 0; i < contact_points.size(); i++) {
            //    U = insertRow(U, contact_points[i]*3+1, Matrix<double, 1, 1>(0));
            //}

            ////cout << "K.inverse():" << endl << K.inverse() << endl;
            //U.resize(U.rows() / 3, 3);
            //cout << "U:" << endl << U << endl;

            //_X += U;
            string path_p = targetpath + "0000" + "/" + it.key() + "_p.msh";
            igl::writeMSH(path_p, _X, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
            
            // =============== end of init rendering ====================

            
            string path_vl = targetpath + "0000" + "/" + it.key() + "_vl.csv";
            string path_vc = targetpath + "0000" + "/" + it.key() + "_vc.csv";
            Xd2csv(path_vl.c_str(), last_vel);
            Xd2csv(path_vc.c_str(), current_vel);
            

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

            MatrixXd dv = XfFromcsv(string(it.value()["deform_velocity"]).c_str());
            MatrixXd rp = XfFromcsv(string(it.value()["rigid_position"]).c_str());
            VectorXd ms = XfFromcsv(string(it.value()["mass"]).c_str());
            MatrixXd YM = XfFromcsv(string(it.value()["Young_s_modulus"]).c_str());

            DynamicObj* dtmp = new DynamicObj_fem(
                it.key(),
                P, 
                Tet, 
                Tri, 
                TriTag, 
                TetTag, 
                XFields, 
                EFields,
                XF, 
                TriF, 
                TetF,
                Eigen::Vector3d(it.value()["linear_velocity"][0], it.value()["linear_velocity"][1], it.value()["linear_velocity"][2]),
                Eigen::Vector3d(it.value()["angular_velocity"][0], it.value()["angular_velocity"][1], it.value()["angular_velocity"][2]),
                dv,
                rp,
                ms,
                -1,
                -1,
                YM
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

void StressFEM::save_scene(int seq)
{
    json jout;

    for (auto i = 0; i < DynamicVec.size(); i++) {
        string path_p = tpath + padseq(seq) + "/" + DynamicVec[i]->name + "_p.msh";
        DynamicVec[i]->writemsh(path_p);
        string path_vl = tpath + padseq(seq) + "/" + DynamicVec[i]->name + "_vl.csv";
        Xf2csv(path_vl.c_str(), lastVec[i]);
        string path_vc = tpath + padseq(seq) + "/" + DynamicVec[i]->name + "_vc.csv";
        Xf2csv(path_vc.c_str(), currentVec[i]);
        string path_x = tpath + padseq(seq) + "/" + DynamicVec[i]->name + "_x.csv";
        Xf2csv(path_x.c_str(), RigidPosVec[i]);
        //debug only
        string path_pp = tpath + padseq(seq) + "/" + DynamicVec[i]->name + "_p.csv";
        Xf2csv(path_pp.c_str(), DynamicVec[i]->get_position());

        //Eigen::Vector3d lv = DynamicVec[i].get_linear_velocity();
        //Eigen::Vector3d av = DynamicVec[i].get_angular_velocity();

        json jtmp;
        jtmp["rigid_position_path"] = path_x;
        jtmp["position_path"] = path_p;
        jtmp["last_velocity"] = path_vl;
        jtmp["current_velocity"] = path_vc;
        jtmp["angular_velocity"] = { 0.0,0.0,0.0};
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

void StressFEM::reset()
{   
    for (auto i = 0; i < DynamicVec.size(); i++)
        delete DynamicVec[i];
    for (auto i = 0; i < StaticVec.size(); i++)
        delete StaticVec[i];
    StaticVec.clear();
    DynamicVec.clear();
    currentVec.clear();
    lastVec.clear();
    RigidPosVec.clear();
    DampVec.clear();

}

void StressFEM::set_physics_params(double restitution, double collision_t, double friction)
{
    e = restitution;
    tc = collision_t;
    u = friction;
}

void StressFEM::run(float delta_t, int seq)
{

}

void StressFEM::rigid(float delta_t, int seq)
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

void StressFEM::fem(float delta_t, int seq)
{
    for (auto i = 0; i < DynamicVec.size(); i++) {
        MatrixXd pl = DynamicVec[i]->get_position();
        //cout << "pl:" << endl << pl << endl;
        MatrixXi tet = DynamicVec[i]->get_tetrahedrons();
        MatrixXd vi = lastVec[i];
        MatrixXd x = RigidPosVec[i];
        double m = DynamicVec[i]->get_mass() / x.rows();

        MatrixXd G = m * gravity.transpose().replicate(x.rows(), 1);
        //cout << "mass_vec: " << endl << mass << endl;
        //MatrixXd g = gravity.transpose().replicate(x.rows(), 1);
        //MatrixXd G = g.array().colwise() * mass.array();
        //cout << "G: " << endl << G << endl;
        /*
        init with:
            pl : deformed position
            tet: tetrahedron indices
            vl : previous velocity
            x  : undeformed position for rigid motion
            m  : mass per tet(uniformly)
            G  : gravitational force vector(uniformly)
        */

        // ============ assemble global stiffness matrix =============

        MatrixXd K;
        int n = pl.rows();// number of vertices
        K.resize(3 * n, 3 * n);
        K.setZero();
        //K.setIdentity();
        //cout << "tet" << endl << tet << endl;
        for (auto i = 0; i < tet.rows(); i++) {
            Matrix<double, 4, 3> teti;
            teti << x.row(tet(i, 0)), x.row(tet(i, 1)), x.row(tet(i, 2)), x.row(tet(i, 3));
            //cout << "teti" << endl << teti << endl;
            MatrixXd ki = TetrahedronElementStiffness(YModulus, teti);
            //cout << "ki" << endl << ki << endl;
            TetrahedronAssemble(K, ki, tet.row(i));
        }

        // ============== get global stiffness matrix =================
        MatrixXd U = pl - x;
        //cout << "pl" << endl << pl << endl;
        //cout << "x" << endl << x << endl;
        //cout << "U: " << endl << U << endl;
        //cout << "U_norm(): " << endl << U.norm() << endl;
        U = mat2vec(U, U.rows() * 3, 1);

        MatrixXd fi = -K * U;

        //cout << "fi: " << endl << vec2mat(fi, x.rows(), 3) << endl;
        //=============== apply boundary conditions =============

        auto [collide, contact_points] = CD_table_FEM(pl);

        MatrixXd f_impact;
        MatrixXd vc = vi;
        /*       cout << "vi: " << endl << vi << endl;*/
             /*  cout << "=============" << endl;
               cout << "seq: " << seq << endl;
               cout << "=============" << endl;*/
               //cout<<pl.row(0)
               //cout <<"collide? " << collide << endl;
        if (collide) {

            for (auto i = 0; i < contact_points.size(); i++) {
                vc.row(contact_points[i]) *= -e;
            }
            /*        cout << "vc: " << endl << vc << endl;*/
            MatrixXd v_y = MatrixXd::Zero(vi.rows(), vi.cols());
            v_y.col(1) = (vc - vi).col(1);
            //MatrixXd fi_tmp = fi;
            MatrixXd fi_tmp = vec2mat(fi, fi.rows() / 3, 3);
            MatrixXd fi_y = MatrixXd::Zero(fi_tmp.rows(), fi_tmp.cols());
            fi_y.col(1) = fi_tmp.col(1);

            /*   cout << "v_y: " << endl << v_y * m / tc << endl;
               cout << "G: " << endl << G << endl;
               cout << "fi_y: " << endl << fi_y << endl;*/

            MatrixXd f_collide = v_y * m / tc - G - fi_y;

            //cout << "f_collide:" << endl << f_collide << endl;
            MatrixXd zero_buff = MatrixXd::Zero(f_collide.rows(), f_collide.cols());

            for (auto i = 0; i < contact_points.size(); i++) {
                zero_buff.row(contact_points[i]) = f_collide.row(contact_points[i]);
            }
            f_impact = zero_buff;
            //cout << "f_impact:" << endl << f_impact << endl;
        }
        //cout << "f_impact(has to be positive): " << endl << f_impact << endl;
        f_impact = mat2vec(f_impact, f_impact.rows() * 3, 1);

        //cout << "fi: " << endl << fi << endl;

        MatrixXd G_i = mat2vec(G, G.rows() * 3, 1);

        //cout << "fi: " << endl << vec2mat(fi, x.rows(), 3) << endl;
        //cout << "f_impact: " << endl << vec2mat(f_impact, x.rows(), 3) << endl;
        //cout << "G_i: " << endl << vec2mat(G_i, x.rows(), 3) << endl;
        MatrixXd f_nodal = fi + f_impact + G_i;

        //cout << "f_nodal: " << endl << vec2mat(f_nodal, x.rows(), 3) << endl;

        MatrixXd fn = f_nodal;
        MatrixXd k = K;

        //cout << x.rows() << endl;

        int index = 0;
        int fd_len = fn.rows();
        while (index < fd_len) {

            if (abs(fn(index, 0)) < 1.0e-8) {
                removeRow(k, index);
                removeColumn(k, index);
                removeRow(fn, index);
                fd_len--;
            }
            else {
                index++;
            }
        }
        //cout << fn.rows() << endl;
        //
        ////cout << "f_deform: " << endl << f_deformation << endl;
        ////cout << "fd: " << endl << fd << endl;
        ////cout << "shape K: " << "(" << K.rows() << ", " << K.cols() << ")" << endl;
        //cout << "shape k: " << "(" << k.rows() << ", " << k.cols() << ")" << endl;
        FullPivLU<MatrixXd> lu_decomp(k);
        auto rank = lu_decomp.rank();
        //cout << "k's rank " << rank << endl;
        //cout << "k" << endl << k << endl;
        //cout << "k.inv" << k.inverse() << endl;
        //cout << "fn" << endl << fn << endl;
        //cout << "check_singular " << (k.determinant() < 1e-8) << endl;
        MatrixXd vf = x;
        vf.setZero();
        //if (k.determinant() > 1e-8) {
        if (true) {
            /* HouseholderQR<MatrixXd> qr(k);
             MatrixXd u = qr.solve(fn);*/

            LeastSquaresConjugateGradient<MatrixXd > lscg;
            lscg.compute(k);
            MatrixXd u = lscg.solve(fn);


            //cout << "u:" << endl << u << endl;
            MatrixXd u_zero;
            u_zero.resize(U.rows(), U.cols());
            u_zero.setZero();

            index = 0;
            for (auto i = 0; i < f_nodal.rows(); i++) {
                //cout <<"fnodal: " << abs(f_nodal(i, 0)) << endl;
                if (abs(f_nodal(i, 0)) > 1e-8) {
                    //cout << u(index) << endl;
                    u_zero(i) = u(index);
                    index++;
                    if (index == u.rows())
                        break;
                }
            }
            //cout << "u_zero:" << endl << vec2mat(u_zero, x.rows(), 3) << endl;
            //====================== applied BC =====================
            /*
            what I have:
            u_zero(padded u);
            compare with d
            */

            vf = vi + vec2mat(f_nodal, x.rows(), 3) / m * delta_t;
            //cout << "f_nodal:" << endl << vec2mat(f_nodal, x.rows(), 3) << endl;
            MatrixXd d = (vi + vf) / 2 * delta_t;
            //cout << "d:" << endl << d << endl;
            //cout << "u_zero:" << endl << vec2mat(u_zero, x.rows(), 3) << endl;
            d = mat2vec(d, d.rows() * 3, 1);
            //cout << d.norm() - u_zero.norm() << endl;
            if (d.norm() - u_zero.norm() > 1e-8) {
                //cout << "hello" << endl;
                d = u_zero;
                //vf = vi + vec2mat(K * u_zero, x.rows(), 3) / m * delta_t;
                // vf = vi + f_nodal/m*t
                // ======= find f_impact ==========
                fi = -K * u_zero;
                fi = vec2mat(fi, x.rows(), 3);
                //MatrixXd fi_tmp = fi
                //cout << "vc: " << endl << vc << endl;
                MatrixXd v_y = MatrixXd::Zero(vi.rows(), vi.cols());
                v_y.col(1) = (vc - vi).col(1);
                //MatrixXd fi_tmp = fi;
                //MatrixXd fi_tmp = vec2mat(fi, fi.rows() / 3, 3);
                MatrixXd fi_y = MatrixXd::Zero(fi.rows(), fi.cols());
                fi_y.col(1) = fi.col(1);

                MatrixXd f_collide = v_y * m / tc - G - fi_y;
                //cout << "f_collide:" << endl << f_collide << endl;
                //cout << "f_collide:" << endl << f_collide << endl;
                MatrixXd zero_buff = MatrixXd::Zero(f_collide.rows(), f_collide.cols());

                for (auto i = 0; i < contact_points.size(); i++) {
                    zero_buff.row(contact_points[i]) = f_collide.row(contact_points[i]);
                }
                f_impact = zero_buff;
                //cout << "f_impact:" << endl << f_impact << endl;

                f_nodal = f_impact + G + fi;
                /*  cout << "f_impact:" << endl << f_impact << endl;
                  cout << "G:" << endl << G << endl;
                  cout << "fi:" << endl << fi << endl;*/
                  //cout << "f_nodal:" << endl << f_nodal << endl;
                  //=== find f_imapct =========
                vf = vi + (f_impact + G + fi) / m * delta_t;
                //cout << "vf:" << endl << vf << endl;
                //vf *= 0.4;
            }
            //cout << "f_nodal:" << endl << vec2mat(f_nodal,x.rows(),3) << endl;
            //cout << "===============" << endl;
            //cout << "f_nodal.norm(): " << f_nodal.norm() << endl;
            //cout << "===============" << endl;
            /*cout << "vf:" << endl << vf << endl;*/
            d = vec2mat(d, x.rows(), 3);

            //cout << "d:" << endl << d << endl;
            pl += d;
        }
        /*  cout << "d:" << endl << d << endl;*/
        DynamicVec[i]->update_state(pl, Vector3d(), Vector3d()); // no w 
        //currentVec[i] = vc + F / m / x.rows() * delta_t;
        lastVec[i] = vf; // doesnot matter
        RigidPosVec[i] = x;
    }
}

