#ifndef GRAPHICSENGINE_H
#define GRAPHICSENGINE_H

#pragma once
#include "Engine.h"
#include <igl/png/writePNG.h>
class GraphicsEngine :
    public Engine
{
private:
    //vector<StaticObj>StaticVec;
    //vector<DynamicObj>DynamicVec;
    string tpath;

public:
    GraphicsEngine();
    GraphicsEngine(string t_path);
    vector<StaticObj> getStaticObjs();
    vector<DynamicObj> getDynamicObjs();
    void load_scene(string folder);
    void save_scene(string t_folder,int seq);
    void reset();
    void run(string project_folder);
};

#endif
