#include "World.h"
#include <sys/stat.h>
#include <fstream>
#include <ctime>
#include <direct.h>
#pragma warning (disable : 4996)

using namespace std;

void make_dir_win(string targetpath, int seq) {
	if(seq<10)
		_mkdir((targetpath + "000" + to_string(seq)).c_str());
	else if(seq<100)
		_mkdir((targetpath + "00" + to_string(seq)).c_str());
	else if(seq<1000)
		_mkdir((targetpath + "0" + to_string(seq)).c_str());
	else
		_mkdir((targetpath + to_string(seq)).c_str());
}

inline bool checkfile(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}


World::~World()
{

}

World::World()
{
}

World::World(string inputpath, string outputpath)
{

	configfilepath = inputpath;
	targetpath = outputpath;
	NewtonRigid PhyEngine;
}

void World::LoadingWorld()
{
	PhyEngine = NewtonRigid(targetpath);
	std::ifstream ifs(configfilepath);
	json j;

	try
	{
		j = json::parse(ifs);
	}
	catch (json::parse_error& ex)
	{
		std::cerr << "parse error at byte " << ex.byte << std::endl;
	}
	currentconfig = j["Objects"];
	
}

void World::init()
{

}

void World::PhysicsRender(float runningtime, int seq)
{	
	make_dir_win(targetpath, seq);

	PhyEngine.load_scene(seq-1);	
	PhyEngine.run(runningtime,seq);
	PhyEngine.save_scene(seq);
	PhyEngine.reset();

}

void World::GraphicsRender(time_t start_time)
{

}


void World::InitConfigs()
{	
	json jout;

	make_dir_win(targetpath, 0);
	Eigen::MatrixXd X;
	Eigen::MatrixXi Tri,Tet;
	Eigen::VectorXi TriTag, TetTag;
	std::vector<std::string> XFields, EFields;
	std::vector<Eigen::MatrixXd> XF,TriF,TetF;

	using namespace Eigen;
	
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
			MatrixXd _V(X.rows(), 3);
			//_V.col(0) = ...
			_V.col(0).setConstant(it.value()["velocity"][0]);
			_V.col(1).setConstant(it.value()["velocity"][1]);
			_V.col(2).setConstant(it.value()["velocity"][2]);
			_X.col(0) = X.col(0);
			_X.col(1) = X.col(1);
			_X.col(2) = X.col(2);
			string path_p = targetpath + "0000" + "/" + it.key() + "_p.msh";
			string path_v = targetpath + "0000" + "/" + it.key() + "_v.msh";
			igl::writeMSH(path_p, _X, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
			igl::writeMSH(path_v, _V, Tri, Tet, TriTag, TetTag, XFields, XF, EFields, TriF, TetF);
			
			// TODO adding initial velocity with .msh file of a complete verision of velocity)

			json jtmp;
			jtmp["position_path"] = path_p;
			jtmp["velocity_path"] = path_v;
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


