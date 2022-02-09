//+
Point(1) = {0, 0, 0, 1.0};
//+
Point(4) = {1, 0, 0, 1.0};
//+
Point(2) = {0, 1, 0, 1.0};
//+
Point(3) = {0, 0, 1, 1.0};

//+
Line(1) = {2, 1};
//+
Line(2) = {2, 4};
//+
Line(3) = {2, 3};
//+
Line(4) = {1, 3};
//+
Line(5) = {1, 4};
//+
Line(6) = {3, 4};
//+
Curve Loop(1) = {3, -4, -1};
//+
Plane Surface(1) = {1};
//+
Curve Loop(2) = {1, 5, -2};
//+
Plane Surface(2) = {2};
//+
Curve Loop(3) = {4, 6, -5};
//+
Plane Surface(3) = {3};
//+
Curve Loop(4) = {3, 6, -2};
//+
Plane Surface(4) = {4};
//+
Surface Loop(1) = {4, 1, 3, 2};
//+
Volume(1) = {1};
