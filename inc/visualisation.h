
#ifndef VISUALISATION_H
#define VISUALISATION_H

#include <igl/opengl/glfw/Viewer.h>
#include <igl/barycenter.h>
#include <igl/colormap.h>

#include <igl/readMSH.h>
#include <igl/readMESH.h>
#include <igl/writeMSH.h>

#include <string>
using namespace std;


void render(string path);
void SaveScene(string scene, string folder);


#endif // 