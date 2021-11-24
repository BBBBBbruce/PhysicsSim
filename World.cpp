#include "World.h"
#include <sys/stat.h>
#pragma warning (disable : 4996)
//#include"ExternalLib/pugixml/pugixml.hpp"

using namespace std;


inline bool checkfile(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

coords string2coords(string input) {
	cout << input << endl;
	char* cstr = input.data();
	char* token = strtok(cstr, ",");
	vector<float> position;
	int i = 0;

	while (token != NULL)
	{
		cout << token << endl;
		position.push_back(atof(token)) ;
		i++;
		token = strtok(NULL, ",");
	}

	coords output = {position[0],position[1],position[2]};
	cout << output.x << output.y << output.z << endl;
	return output;
}

World::~World()
{
	//delete[] currentconfig;
	//delete[] outputconfig;
}

World::World()
{
	//currentconfig = new xml();
	//outputconfig = new xml();
}

World::World(string filepath)
{
	configfilepath = filepath;

	//currentconfig = new xml();
	//outputconfig = new xml();
}

void World::LoadingWorld()
{
	//loading objs
	std::cout << "loading world" << std::endl;
	if (!checkfile(configfilepath)){
		throw std::runtime_error(std::string("Failed to load setup file "));
	}
	std::cout << "reading config" << std::endl;

	pugi::xml_document file;
	pugi::xml_parse_result result;

	result = file.load_file(configfilepath.c_str());
	currentconfig = file.child("worldobject");
	//NewtonRigid PhyEngine(this->currentconfig);
	//cout << currentconfig.child("object").attribute("category").value() << endl;
	vector<DynamicObj> dvec;
	vector<StaticObj> svec;
	// have to parse it here, no idea what is going on
	
	for (pugi::xml_node_iterator it = currentconfig.begin(); it != currentconfig.end(); ++it)
	{
		std::cout << "items:" << it->attribute("category").value() << endl;
		if (string(it->attribute("category").value()) == "dynamic" ){

		}
		else if (string(it->attribute("category").value()) == "static") {
			cout << "hello" << endl;
			cout << it->child("name").child_value() << endl;
			cout << "hello" << endl;
			cout << it->child("position").child_value() << endl;
			cout << "hello" << endl;
			cout << it->child("path").child_value() << endl;

			coords pos = string2coords(string(it->child("position").child_value()));

			StaticObj statictmp(string(it->child("name").child_value()),
				string(it->child("path").child_value()),
				pos);
		}

	}

	
}

void World::PhysicsRender()
{
	//add augments
	std::cout << configfilepath << endl;
	//if(*this->currentconfig!=NULL)
	//	NewtonRigid PhyEngine(*this->currentconfig);

	//std::cout << "hello" << std::endl;
	//while(this->currentconfig!=NULL)
	//	cout << currentconfig.child("object").attribute("category").value() << endl;
	//PhyEngine.ParseWorld();
	//PhyEngine.run(1);
	//outputconfig = PhyEngine.ExportWorld();
	//outputconfigfile();
}

void World::GraphicsRender()
{
}

void World::outputconfigfile()
{
	//save tree to xml
	//std::cout << "Saving result: " << outputconfig.save_file("bin/save_file_output.xml") << std::endl;
}


