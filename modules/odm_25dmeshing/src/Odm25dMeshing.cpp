#include "Odm25dMeshing.hpp"
#include <CGAL/Shape_detection_3.h>
#include <CGAL/regularize_planes.h>

int Odm25dMeshing::run(int argc, char **argv) {
	log << logFilePath << "\n";

	// If no arguments were passed, print help and return early.
	if (argc <= 1) {
		printHelp();
		return EXIT_SUCCESS;
	}

	try {

		parseArguments(argc, argv);

		loadPointCloud();

		savePlanes();
		//buildMesh();

	} catch (const Odm25dMeshingException& e) {
		log.setIsPrintingInCout(true);
		log << e.what() << "\n";
		log.printToFile(logFilePath);
		log << "For more detailed information, see log file." << "\n";
		return EXIT_FAILURE;
	} catch (const std::exception& e) {
		log.setIsPrintingInCout(true);
		log << "Error in OdmMeshing:\n";
		log << e.what() << "\n";
		log.printToFile(logFilePath);
		log << "For more detailed information, see log file." << "\n";
		return EXIT_FAILURE;
	}

	log.printToFile(logFilePath);

	return EXIT_SUCCESS;
}

void Odm25dMeshing::parseArguments(int argc, char **argv) {
	for (int argIndex = 1; argIndex < argc; ++argIndex) {
		// The argument to be parsed.
		std::string argument = std::string(argv[argIndex]);

		if (argument == "-help") {
			printHelp();
			exit(0);
		} else if (argument == "-verbose") {
			log.setIsPrintingInCout(true);
		} else if (argument == "-maxVertexCount" && argIndex < argc) {
            ++argIndex;
            if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
            std::stringstream ss(argv[argIndex]);
            ss >> maxVertexCount;
            if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
            maxVertexCount = std::max<unsigned int>(maxVertexCount, 0);
            log << "Vertex count was manually set to: " << maxVertexCount << "\n";
		} else if (argument == "-wlopIterations" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
			std::stringstream ss(argv[argIndex]);
			ss >> wlopIterations;
			if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");

			wlopIterations = std::min<unsigned int>(1000, std::max<unsigned int>(wlopIterations, 1));
			log << "WLOP iterations was manually set to: " << wlopIterations << "\n";
		} else if (argument == "-inputFile" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) {
				throw Odm25dMeshingException(
						"Argument '" + argument
								+ "' expects 1 more input following it, but no more inputs were provided.");
			}
			inputFile = std::string(argv[argIndex]);
			std::ifstream testFile(inputFile.c_str(), std::ios::binary);
			if (!testFile.is_open()) {
				throw Odm25dMeshingException(
						"Argument '" + argument	+ "' has a bad value. (file not accessible)");
			}
			testFile.close();
			log << "Reading point cloud at: " << inputFile << "\n";
		} else if (argument == "-outputFile" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) {
				throw Odm25dMeshingException(
						"Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
			}
			outputFile = std::string(argv[argIndex]);
			std::ofstream testFile(outputFile.c_str());
			if (!testFile.is_open()) {
				throw Odm25dMeshingException(
						"Argument '" + argument + "' has a bad value.");
			}
			testFile.close();
			log << "Writing output to: " << outputFile << "\n";
		} else if (argument == "-logFile" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) {
				throw Odm25dMeshingException(
						"Argument '" + argument
								+ "' expects 1 more input following it, but no more inputs were provided.");
			}
			logFilePath = std::string(argv[argIndex]);
			std::ofstream testFile(outputFile.c_str());
			if (!testFile.is_open()) {
				throw Odm25dMeshingException(
						"Argument '" + argument + "' has a bad value.");
			}
			testFile.close();
			log << "Writing log information to: " << logFilePath << "\n";
		} else {
			printHelp();
			throw Odm25dMeshingException("Unrecognised argument '" + argument + "'");
		}
	}
}

void Odm25dMeshing::loadPointCloud(){
	  PlyInterpreter interpreter(points, normals);

	  std::ifstream in(inputFile);
	  if (!in || !CGAL::read_ply_custom_points (in, interpreter, Kernel())){
		  throw Odm25dMeshingException(
		  				"Error when reading points and normals from:\n" + inputFile + "\n");
	  }

	  flipFaces = interpreter.flip_faces();

	  log << "Successfully loaded " << points.size() << " points from file\n";
}

void Odm25dMeshing::savePlanes(){
	typedef std::pair<Point3, Vector3>         Point_with_normal;
	typedef std::vector<Point_with_normal>     Pwn_vector;
	typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
	typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
	typedef CGAL::Shape_detection_3::Efficient_RANSAC_traits<Kernel, Pwn_vector, Point_map, Normal_map>            Traits;
	typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>   Efficient_ransac;
	typedef CGAL::Shape_detection_3::Plane<Traits>              Plane;
	typedef CGAL::Shape_detection_3::Cone<Traits>             Cone;
	typedef CGAL::Shape_detection_3::Cylinder<Traits>         Cylinder;
	typedef CGAL::Shape_detection_3::Sphere<Traits>           Sphere;

//	My_point_property_map ppmap(points);
//	Tree tree(
//		boost::counting_iterator<std::size_t>(0),
//		boost::counting_iterator<std::size_t>(points.size()),
//		Splitter(),
//		TreeTraits(ppmap)
//	  );
//
	Pwn_vector output;
//	const unsigned int K = 8;
	const float NORMAL_THRESHOLD = 0.92;
//
//	Distance tr_dist(ppmap);
//
//	// Initialize the search structure, and search all N points
//	for (size_t i = 0; i < points.size(); i++){
//	  Point3 &currentPoint = points[i];
//	  Vector3 &currentNormal = normals[i];
//	  unsigned short sameDirection = 0;
//
//	  Neighbor_search search(tree, currentPoint, K, 0, true, tr_dist);
//
//	  for(Neighbor_search::iterator it = search.begin(); it != search.end(); it++){
//		  Vector3 &n = normals[it->first];
//
//		  FT dotProduct = currentNormal.x() * n.x() +
//						  currentNormal.y() * n.y() +
//						  currentNormal.z() * n.z();
//
//		  if (dotProduct >= NORMAL_THRESHOLD) sameDirection++;
//		  else break;
//	  }
//
//
//	  if (sameDirection == K){
//		  // Likely planar surface
//		  output.push_back(std::make_pair(currentPoint, currentNormal));
//	  }
//	}

	for (size_t i = 0; i < points.size(); i++){
		  Point3 &currentPoint = points[i];
		  Vector3 &currentNormal = normals[i];
		output.push_back(std::make_pair(currentPoint, currentNormal));
	}
	// Instantiates shape detection engine.
	Efficient_ransac ransac;
	// Provides the input data.
	ransac.set_input(output);
	// Registers detection of planes
	ransac.add_shape_factory<Plane>();
//	ransac.add_shape_factory<Cone>();
//	ransac.add_shape_factory<Cylinder>();
//	ransac.add_shape_factory<Sphere>();


	// Sets parameters for shape detection.
	Efficient_ransac::Parameters parameters;
	// Sets probability to miss the largest primitive at each iteration.
	parameters.probability = 0.05;

	// Detect shapes with at least 500 points.
	parameters.min_points = 500;

	// Sets maximum Euclidean distance between a point and a shape.
	parameters.epsilon = 0.15;

	// Sets maximum Euclidean distance between points to be clustered.
	parameters.cluster_epsilon = 0.15;

	// Sets maximum normal deviation.
	// 0.9 < dot(surface_normal, point_normal);
	parameters.normal_threshold = NORMAL_THRESHOLD;

	// Detects shapes
	ransac.detect(parameters);

	// Prints number of detected shapes and unassigned points.
	std::cout << ransac.shapes().end() - ransac.shapes().begin() << " detected shapes, "
	 << ransac.number_of_unassigned_points()
	 << " unassigned points." << std::endl;

	// Efficient_ransac::shapes() provides
	// an iterator range to the detected shapes.
	Efficient_ransac::Shape_range shapes = ransac.shapes();
	Efficient_ransac::Shape_range::iterator it = shapes.begin();

	std::filebuf fb;
	fb.open(outputFile, std::ios::out);
	std::ostream os(&fb);

	os << "ply\n"
	 << "format ascii 1.0\n"
	 << "element vertex " << output.size() - ransac.number_of_unassigned_points() << "\n"
	 << "property float x\n"
	 << "property float y\n"
	 << "property float z\n"
	 << "element face 0\n"
	 << "property list uchar int vertex_index\n"
	 << "end_header\n";

	while (it != shapes.end()) {
		boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
		// Using Shape_base::info() for printing
		// the parameters of the detected shape.
		std::cout << (*it)->info() << std::endl;


		// Iterates through point indices assigned to each detected shape.
		std::vector<std::size_t>::const_iterator
		  index_it = (*it)->indices_of_assigned_points().begin();
		while (index_it != (*it)->indices_of_assigned_points().end()) {
		  // Retrieves point
		  const Point_with_normal &p = *(output.begin() + (*index_it));
		  os << p.first.x() << " " << p.first.y() << " " << p.first.z() << std::endl;
		  index_it++;
		}

		// Proceeds with next detected shape.
		it++;
	}

	std::cout << "Done" << std::endl;
}

void Odm25dMeshing::buildMesh(){
	size_t pointCount = points.size();

	const double RETAIN_PERCENTAGE = std::min<double>(80., 100. * static_cast<double>(maxVertexCount) / static_cast<double>(pointCount));   // percentage of points to retain.
	std::vector<Point3> simplifiedPoints;

	log << "Performing weighted locally optimal projection simplification and regularization (retain: " << RETAIN_PERCENTAGE << "%, iterate: " << wlopIterations << ")" << "\n";

	CGAL::wlop_simplify_and_regularize_point_set<Concurrency_tag>(
		 	points.begin(),
			points.end(),
			std::back_inserter(simplifiedPoints),
			RETAIN_PERCENTAGE,
			-1,
			wlopIterations,
			false);

	pointCount = simplifiedPoints.size();

	if (pointCount < 3){
		throw Odm25dMeshingException("Not enough points");
	}

	log << "Vertex count is " << pointCount << "\n";

	typedef CDT::Point cgalPoint;
	std::vector< std::pair<cgalPoint, size_t > > pts;
	try{
		pts.reserve(pointCount);
	} catch (const std::bad_alloc&){
		throw Odm25dMeshingException("Not enough memory");
	}

	for (size_t i = 0; i < pointCount; ++i){
		pts.push_back(std::make_pair(cgalPoint(simplifiedPoints[i].x(), simplifiedPoints[i].y()), i));
	}

	log << "Computing delaunay triangulation... ";

	CDT cdt;
	cdt.insert(pts.begin(), pts.end());

	unsigned int numberOfTriangles = static_cast<unsigned >(cdt.number_of_faces());
	unsigned int triIndexes = numberOfTriangles*3;

	if (numberOfTriangles == 0) throw Odm25dMeshingException("No triangles in resulting mesh");

	log << numberOfTriangles << " triangles\n";

	std::vector<float> vertices;
	std::vector<int> vertexIndices;

	try{
		vertices.reserve(pointCount);
		vertexIndices.reserve(triIndexes);
	} catch (const std::bad_alloc&){
		throw Odm25dMeshingException("Not enough memory");
	}


	log << "Saving mesh to file.\n";

	std::filebuf fb;
	fb.open(outputFile, std::ios::out);
	std::ostream os(&fb);

	os << "ply\n"
	   << "format ascii 1.0\n"
	   << "element vertex " << pointCount << "\n"
	   << "property float x\n"
	   << "property float y\n"
	   << "property float z\n"
	   << "element face " << numberOfTriangles << "\n"
	   << "property list uchar int vertex_index\n"
	   << "end_header\n";

	for (size_t i = 0; i < pointCount; ++i){
		os << simplifiedPoints[i].x() << " " << simplifiedPoints[i].y() << " " << simplifiedPoints[i].z() << std::endl;
	}

	for (CDT::Face_iterator face = cdt.faces_begin(); face != cdt.faces_end(); ++face) {
		os << 3 << " ";

		if (flipFaces){
			os << face->vertex(2)->info() << " " << face->vertex(1)->info() << " " << face->vertex(0)->info() << std::endl;
		}else{
			os << face->vertex(0)->info() << " " << face->vertex(1)->info() << " " << face->vertex(2)->info() << std::endl;
		}
	}

	fb.close();

	log << "Successfully wrote mesh to: " << outputFile << "\n";
}

void Odm25dMeshing::printHelp() {
	bool printInCoutPop = log.isPrintingInCout();
	log.setIsPrintingInCout(true);

	log << "Usage: odm_25dmeshing -inputFile [plyFile] [optional-parameters]\n";
	log << "Create a 2.5D mesh from an oriented point cloud (points with normals) using a constrained delaunay triangulation. "
		<< "The program requires a path to an input PLY point cloud file, all other input parameters are optional.\n\n";

	log << "	-inputFile	<path>	to PLY point cloud\n"
		<< "	-outputFile	<path>	where the output PLY 2.5D mesh should be saved (default: " << outputFile << ")\n"
		<< "	-logFile	<path>	log file path (default: " << logFilePath << ")\n"
		<< "	-verbose	whether to print verbose output (default: " << (printInCoutPop ? "true" : "false") << ")\n"
		<< "	-maxVertexCount	<0 - N>	Maximum number of vertices in the output mesh. The mesh might have fewer vertices, but will not exceed this limit. (default: " << maxVertexCount << ")\n"
		<< "	-wlopIterations	<1 - 1000>	Iterations of the Weighted Locally Optimal Projection (WLOP) simplification algorithm. Higher values take longer but produce a smoother mesh. (default: " << wlopIterations << ")\n"

		<< "\n";

	log.setIsPrintingInCout(printInCoutPop);
}



