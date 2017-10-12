#include "PlyInterpreter.hpp"

// Init and test if input file contains the right properties
bool PlyInterpreter::is_applicable(CGAL::Ply_reader& reader) {
	return reader.does_tag_exist<FT> ("x")
	      && reader.does_tag_exist<FT> ("y")
	      && reader.does_tag_exist<FT> ("z")
		  && reader.does_tag_exist<FT> ("nx")
		  && reader.does_tag_exist<FT> ("ny")
		  && reader.does_tag_exist<FT> ("nz");
}

// Describes how to process one line (= one point object)
void PlyInterpreter::process_line(CGAL::Ply_reader& reader) {
	FT x = (FT)0., y = (FT)0., z = (FT)0.,
		nx = (FT)0., ny = (FT)0., nz = (FT)0.;
	unsigned char classification = 2;
	const char CLASS_GROUND = 2;

	reader.assign (x, "x");
	reader.assign (y, "y");
	reader.assign (z, "z");
	reader.assign (nx, "nx");
	reader.assign (ny, "ny");
	reader.assign (nz, "nz");

	if (reader.does_tag_exist<unsigned char>("classification")){
		reader.assign(classification, "classification");
	}else if (!warnedClassificationMissing){
		std::cout << "WARNING: Points are missing the classification tag, will treat all points as ground.\n";
		warnedClassificationMissing = true;
	}

	Point3 p(x, y, z);
	Vector3 n(nx, ny, nz);

	if (nz >= 0 && zNormalsDirectionCount < std::numeric_limits<long>::max()){
		zNormalsDirectionCount++;
	}else if (nz < 0 && zNormalsDirectionCount > std::numeric_limits<long>::min()){
		zNormalsDirectionCount--;
	}

	if (classification == CLASS_GROUND){
		groundPoints.push_back(p);
		groundNormals.push_back(n);
	}else{
		nongroundPoints.push_back(p);
		nongroundNormals.push_back(n);
	}
}

bool PlyInterpreter::flip_faces(){
	return zNormalsDirectionCount < 0;
}
