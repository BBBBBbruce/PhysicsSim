
#include<iostream>
#include<string>

using namespace std;


void test(string path) {
	pugi::xml_document doc;

	pugi::xml_parse_result result = doc.load_file(path.c_str());

	std::cout << "Load result: " << result.description() << ", objects:  " << doc.child("worldobject").child("object").attribute("category").value() << std::endl;
}