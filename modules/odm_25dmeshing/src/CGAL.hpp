#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Triangulation_2.h>

#include <CGAL/remove_outliers.h>
#include <CGAL/Shape_detection_3.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point3;
typedef Kernel::Vector_3 Vector3;

//We define a vertex_base with info. The "info" (size_t) allow us to keep track of the original point index.
typedef CGAL::Triangulation_vertex_base_with_info_2<size_t, Kernel> Vb;
typedef CGAL::Constrained_triangulation_face_base_2<Kernel> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel, Tds> CDT;

typedef std::pair<Point3, Vector3>         Point_with_normal;
typedef std::vector<Point_with_normal>     Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
typedef CGAL::Shape_detection_3::Efficient_RANSAC_traits<Kernel, Pwn_vector, Point_map, Normal_map>            Traits;
typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>   Efficient_ransac;
typedef CGAL::Shape_detection_3::Plane<Traits>              Plane;

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif


