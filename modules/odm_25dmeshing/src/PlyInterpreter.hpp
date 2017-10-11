#pragma once

#include <utility>
#include <vector>
#include <fstream>
#include <limits>

#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>

#include "CGAL.hpp"

class PlyInterpreter {
	std::vector<Point3>& points;
	std::vector<Vector3>& normals;

	long zNormalsDirectionCount;

	public:
	 PlyInterpreter (std::vector<Point3>& points, std::vector<Vector3>& normals)
	    : points (points), normals(normals), zNormalsDirectionCount(0)
	  { }
	  bool is_applicable (CGAL::Ply_reader& reader);
	  void process_line (CGAL::Ply_reader& reader);
	  bool flip_faces();
};
