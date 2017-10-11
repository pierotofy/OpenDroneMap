#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Triangulation_2.h>

#include <CGAL/point_generators_3.h>
#include <CGAL/Orthogonal_k_neighbor_search.h>
#include <CGAL/Search_traits_3.h>
#include <CGAL/Search_traits_adapter.h>
#include <CGAL/remove_outliers.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point3;
typedef Kernel::Vector_3 Vector3;

//We define a vertex_base with info. The "info" (size_t) allow us to keep track of the original point index.
typedef CGAL::Triangulation_vertex_base_with_info_2<size_t, Kernel> Vb;
typedef CGAL::Constrained_triangulation_face_base_2<Kernel> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel, Tds> CDT;

//definition of a non-mutable lvalue property map,
//with the get function as a friend function to give it
//access to the private member
class My_point_property_map{
  const std::vector<Point3>& points;
public:
  typedef Point3 value_type;
  typedef const value_type& reference;
  typedef std::size_t key_type;
  typedef boost::lvalue_property_map_tag category;
  My_point_property_map(const std::vector<Point3>& pts):points(pts){}
  reference operator[](key_type k) const {return points[k];}
  friend reference get(const My_point_property_map& ppmap,key_type i)
  {return ppmap[i];}
};

typedef CGAL::Search_traits_3<Kernel> BaseTraits;
typedef CGAL::Search_traits_adapter<std::size_t,My_point_property_map,BaseTraits> TreeTraits;
typedef CGAL::Orthogonal_k_neighbor_search<TreeTraits> Neighbor_search;
typedef Neighbor_search::Tree Tree;
typedef Tree::Splitter Splitter;
typedef Neighbor_search::Distance Distance;

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

//typedef CGAL::First_of_pair_property_map<Pwn> Point_map;
//typedef CGAL::Second_of_pair_property_map<Pwn> Normal_map;

