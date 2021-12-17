
#ifndef VISUALISATION_H
#define VISUALISATION_H



#include <igl/opengl/glfw/Viewer.h>
#include <igl/barycenter.h>
#include <igl/colormap.h>

#include <igl/readMSH.h>
#include <igl/readMESH.h>
#include <igl/writeMSH.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_3 Point;
typedef K::Plane_3 Plane;
typedef K::Vector_3 Vector;
typedef K::Segment_3 Segment;
typedef K::Ray_3 Ray;
typedef CGAL::Polyhedron_3<K> Polyhedron;
typedef CGAL::AABB_face_graph_triangle_primitive<Polyhedron> Primitive;
typedef CGAL::AABB_traits<K, Primitive> Traits;
typedef CGAL::AABB_tree<Traits> Tree;
typedef boost::optional< Tree::Intersection_and_primitive_id<Segment>::Type > Segment_intersection;
typedef boost::optional< Tree::Intersection_and_primitive_id<Plane>::Type > Plane_intersection;
typedef Tree::Primitive_id Primitive_id;

#include <string>
using namespace std;


void render(string path);
void SaveScene(string scene, string folder);
string padseq(int seq);


#endif // 