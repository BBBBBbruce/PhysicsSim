// Gmsh project created on Wed Feb 09 11:24:45 2022
SetFactory("OpenCASCADE");
//+
Point(1) = {0, 1, 0, 1.0};
//+
Point(2) = {0, 0, 1, 1.0};
//+
Point(3) = {-0.866, 0, -0.5, 1.0};
//+
Point(4) = {0.866, 0, -0.5, 1.0};
//+
Line(1) = {2, 1};
//+
Line(2) = {1, 4};
//+
Line(3) = {1, 3};
//+
Line(4) = {2, 3};
//+
Line(5) = {2, 4};
//+
Line(6) = {3, 4};
//+
Curve Loop(1) = {2, -5, 1};
//+
Plane Surface(1) = {1};
//+
Curve Loop(2) = {3, -4, 1};
//+
Plane Surface(2) = {2};
//+
Curve Loop(3) = {2, -6, -3};
//+
Plane Surface(3) = {3};
//+
Curve Loop(4) = {5, -6, -4};
//+
Plane Surface(4) = {4};
//+
Surface Loop(1) = {3, 1, 4, 2};
//+
Volume(1) = {1};
