#pragma once

#include <utility>
#include <vector>
#include <fstream>
#include <limits>

#include <CGAL/property_map.h>
#include <CGAL/IO/read_ply_points.h>

#include "CGAL.hpp"

class PlyInterpreter {
	std::vector<Point3>& groundPoints;
	std::vector<Vector3>& groundNormals;
	std::vector<Point3>& nongroundPoints;
	std::vector<Vector3>& nongroundNormals;

	long zNormalsDirectionCount;

	bool warnedClassificationMissing = false;

	public:
	 PlyInterpreter (std::vector<Point3>& groundPoints,
			 	 	 std::vector<Vector3>& groundNormals,
					 std::vector<Point3>& nongroundPoints,
					 std::vector<Vector3>& nongroundNormals)
	    : groundPoints (groundPoints), groundNormals(groundNormals),
		  nongroundPoints(nongroundPoints), nongroundNormals(nongroundNormals),
		  zNormalsDirectionCount(0)
	  { }
	  bool is_applicable (CGAL::Ply_reader& reader);
	  void process_line (CGAL::Ply_reader& reader);
	  bool flip_faces();
};
