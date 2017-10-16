#include "Odm25dMeshing.hpp"
#include <CGAL/grid_simplify_point_set.h>

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

		detectPlanes();
		buildMesh();

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
	  PlyInterpreter interpreter(groundPoints, groundNormals, nongroundPoints, nongroundNormals);

	  std::ifstream in(inputFile);
	  if (!in || !CGAL::read_ply_custom_points (in, interpreter, Kernel())){
		  throw Odm25dMeshingException(
		  				"Error when reading points and normals from:\n" + inputFile + "\n");
	  }

	  flipFaces = interpreter.flip_faces();

	  log << "Loaded " << groundPoints.size() << " ground points\n";
	  log << "Loaded " << nongroundPoints.size() << " non-ground points\n";
}

// Detect planes from non-ground points
// and place them into the ground points (this just to avoid
// creating a new vector)
void Odm25dMeshing::detectPlanes(){
	if (nongroundPoints.size() < 3) return;

	Pwn_vector input;
	const float NORMAL_THRESHOLD = 0.90;

	log << "Computing non-ground points average spacing... ";

	FT avgSpacing = CGAL::compute_average_spacing<Concurrency_tag>(
			nongroundPoints.begin(),
			nongroundPoints.end(),
			24);

	log << avgSpacing << "\n";

	for (size_t i = 0; i < nongroundPoints.size(); i++){
		input.push_back(std::make_pair(nongroundPoints[i], nongroundNormals[i]));
	}

	log << "Detecting planar surfaces..." << "\n";

	// Instantiates shape detection engine.
	Efficient_ransac ransac;
	// Provides the input data.
	ransac.set_input(input);
	// Registers detection of planes
	ransac.add_shape_factory<Plane>();

	// Sets parameters for shape detection.
	Efficient_ransac::Parameters parameters;
	// Sets probability to miss the largest primitive at each iteration.
	parameters.probability = 0.05;

	// Detect shapes with at least 500 points
	parameters.min_points = 500;

	// Sets maximum Euclidean distance between a point and a shape.
	parameters.epsilon = avgSpacing / 2.0;

	// Sets maximum Euclidean distance between points to be clustered.
	parameters.cluster_epsilon = avgSpacing / 2.0;

	// Sets maximum normal deviation.
	// NORMAL_THRESHOLD < dot(surface_normal, point_normal);
	parameters.normal_threshold = NORMAL_THRESHOLD;

	// Detects shapes
	ransac.detect(parameters);

	// Prints number of detected shapes and unassigned points.
	log << ransac.shapes().end() - ransac.shapes().begin() << " detected shapes, "
	 << ransac.number_of_unassigned_points()
	 << " unassigned points.\n";

	// Efficient_ransac::shapes() provides
	// an iterator range to the detected shapes.
	Efficient_ransac::Shape_range shapes = ransac.shapes();
	Efficient_ransac::Shape_range::iterator it = shapes.begin();

	while (it != shapes.end()) {
		boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
		// Using Shape_base::info() for printing
		// the parameters of the detected shape.
		log << (*it)->info() << "\n";

		// Iterates through point indices assigned to each detected shape.
		std::vector<std::size_t>::const_iterator
		  index_it = (*it)->indices_of_assigned_points().begin();
		while (index_it != (*it)->indices_of_assigned_points().end()) {
		  // Retrieves point
		  const Point_with_normal &p = *(input.begin() + (*index_it));
		  groundPoints.push_back(p.first);
		  groundNormals.push_back(p.second);
		  index_it++;
		}

		// Proceeds with next detected shape.
		it++;
	}
}

void Odm25dMeshing::buildMesh(){
	size_t pointCount = groundPoints.size();

	log << "Computing points average spacing... ";

	FT avgSpacing = CGAL::compute_average_spacing<Concurrency_tag>(
			groundPoints.begin(),
			groundPoints.end(),
			24);

	log << avgSpacing << "\n";

	log << "Grid Z sampling and smoothing... ";

	size_t pointCountBeforeGridSampling = pointCount;

	double gridStep = avgSpacing / 2.0;
	Kernel::Iso_cuboid_3 bbox = CGAL::bounding_box(groundPoints.begin(), groundPoints.end());
	Vector3 boxDiag = bbox.max() - bbox.min();

	int gridWidth = 1 + static_cast<unsigned>(boxDiag.x() / gridStep + 0.5);
	int gridHeight = 1 + static_cast<unsigned>(boxDiag.y() / gridStep + 0.5);

	#define KEY(i, j) (i * gridWidth + j)

	std::unordered_map<int, Point_with_normal> grid;

	for (size_t c = 0; c < pointCount; c++){
		const Point_with_normal &pwn = std::make_pair(groundPoints[c], groundNormals[c]);
		Vector3 relativePos = pwn.first - bbox.min();
		int i = static_cast<int>((relativePos.x() / gridStep + 0.5));
		int j = static_cast<int>((relativePos.y() / gridStep + 0.5));

		if ((i >= 0 && i < gridWidth) && (j >= 0 && j < gridHeight)){
			int key = KEY(i, j);

			if (grid.find(key) == grid.end()){
				grid[key] = pwn;
			}else if ((!flipFaces && pwn.first.z() > grid[key].first.z()) || (flipFaces && pwn.first.z() < grid[key].first.z())){
				grid[key] = pwn;
			}
		}
	}

	std::vector<FT> bucket;
	unsigned int smoothedPoints = 0;

	for (int i = 1; i < gridWidth - 1; i++){
		for (int j = 1; j < gridHeight - 1; j++){
			int key = KEY(i, j);

			if (grid.find(key) != grid.end()){
				const Point_with_normal &pwn = grid[key];

				for (int ni = i - 1; ni < i + 2; ni++){
					for (int nj = j - 1; nj < j + 2; nj++){
						if (ni == i && nj == j) continue;
						int nkey = KEY(ni, nj);

						if (grid.find(nkey) != grid.end()) bucket.push_back(grid[nkey].first.z());
					}
				}

				if (bucket.size() >= 5){
					FT mean = accumulate(bucket.begin(), bucket.end(), 0.0) / bucket.size();
					FT variance = 0.0;

					for (unsigned int k = 0; k < bucket.size(); k++) variance += fabs(bucket[k] - mean);
					variance /= bucket.size();

					if (fabs(pwn.first.z() - mean) >= 3 * variance){
						grid.erase(key);
						smoothedPoints++;
					}
				}
			}

			bucket.clear();
		}
	}

	std::vector<Point_with_normal> gridPoints;
	std::vector<Point3> gridPointsWithoutNormals;

	for ( auto it = grid.begin(); it != grid.end(); ++it ){
		gridPoints.push_back(it->second);
		gridPointsWithoutNormals.push_back(it->second.first);
	}

	pointCount = gridPoints.size();
	log << "smoothed " << smoothedPoints << " points, sampled " << pointCount << " points\n";

//	const double RETAIN_PERCENTAGE = std::min<double>(80., 100. * static_cast<double>(maxVertexCount) / static_cast<double>(pointCount));   // percentage of points to retain.
//	std::vector<Point_with_normal> simplifiedPoints;
//
//	log << "Performing weighted locally optimal projection simplification and regularization (retain: " << RETAIN_PERCENTAGE << "%, iterate: " << wlopIterations << ")" << "\n";
//
//	CGAL::wlop_simplify_and_regularize_point_set<Concurrency_tag>(
//			gridPoints.begin(),
//			gridPoints.end(),
//			std::back_inserter(simplifiedPoints),
//			CGAL::First_of_pair_property_map<Point_with_normal>(),
//			RETAIN_PERCENTAGE,
//			-1,
//			wlopIterations,
//			true);
//
//	pointCount = simplifiedPoints.size();
//
//	if (pointCount < 3){
//		throw Odm25dMeshingException("Not enough points");
//	}
//
//	log << "Vertex count is " << pointCount << "\n";

//	avgSpacing = CGAL::compute_average_spacing<Concurrency_tag>(
//			simplifiedPoints.begin(),
//			simplifiedPoints.end(),
//		24);

	  size_t pointBeforeGridSimplify = gridPoints.size();
	  gridPoints.erase(CGAL::grid_simplify_point_set(gridPoints.begin(), gridPoints.end(),
			  CGAL::First_of_pair_property_map<Point_with_normal>(),
			  avgSpacing * 2),
			  gridPoints.end());
	  log << "Removed " << (pointBeforeGridSimplify - gridPoints.size()) << " points with grid simplification\n";

	Polyhedron poly;

	if (!CGAL::poisson_surface_reconstruction_delaunay
	      (gridPoints.begin(), gridPoints.end(),
	       CGAL::First_of_pair_property_map<Point_with_normal>(),
	       CGAL::Second_of_pair_property_map<Point_with_normal>(),
	       poly, avgSpacing))
	{
		throw Odm25dMeshingException("Failed to generate poisson surface reconstruction");
	}

    typedef typename Polyhedron::Vertex_const_iterator VCI;
    typedef typename Polyhedron::Facet_const_iterator FCI;
    typedef typename Polyhedron::Halfedge_around_facet_const_circulator HFCC;

	std::filebuf fb;
	fb.open(outputFile, std::ios::out);
	std::ostream os(&fb);

	os << "ply\n"
	   << "format ascii 1.0\n"
	   << "element vertex " << poly.size_of_vertices() << "\n"
	   << "property float x\n"
	   << "property float y\n"
	   << "property float z\n"
	   << "element face " << poly.size_of_facets() << "\n"
	   << "property list uchar int vertex_index\n"
	   << "end_header\n";

	for (auto it = poly.vertices_begin(); it != poly.vertices_end(); it++){
		os << it->point().x() << " " << it->point().y() << " " << it->point().z() << std::endl;
	}

	typedef CGAL::Inverse_index<VCI> Index;
	Index index(poly.vertices_begin(), poly.vertices_end());

	std::stack<int> s;

	for( FCI fi = poly.facets_begin(); fi != poly.facets_end(); ++fi) {
		HFCC hc = fi->facet_begin();
		HFCC hc_end = hc;

		os << circulator_size(hc) << " ";
		do {
			s.push(index[VCI(hc->vertex())]);
			++hc;
		} while( hc != hc_end);

		while(!s.empty()){
			os << s.top() << " ";
			s.pop();
		}

		os << "\n";
	}

	fb.close();

//
//	typedef CDT::Point cgalPoint;
//	std::vector< std::pair<cgalPoint, size_t > > pts;
//	try{
//		pts.reserve(pointCount);
//	} catch (const std::bad_alloc&){
//		throw Odm25dMeshingException("Not enough memory");
//	}
//
//	for (size_t i = 0; i < pointCount; ++i){
//		pts.push_back(std::make_pair(cgalPoint(simplifiedPoints[i].x(), simplifiedPoints[i].y()), i));
//	}
//
//	log << "Computing delaunay triangulation... ";
//
//	CDT cdt;
//	cdt.insert(pts.begin(), pts.end());
//
//	unsigned int numberOfTriangles = static_cast<unsigned >(cdt.number_of_faces());
//	unsigned int triIndexes = numberOfTriangles*3;
//
//	if (numberOfTriangles == 0) throw Odm25dMeshingException("No triangles in resulting mesh");
//
//	log << numberOfTriangles << " triangles\n";
//
//	std::vector<float> vertices;
//	std::vector<int> vertexIndices;
//
//	try{
//		vertices.reserve(pointCount);
//		vertexIndices.reserve(triIndexes);
//	} catch (const std::bad_alloc&){
//		throw Odm25dMeshingException("Not enough memory");
//	}
//
//
//	log << "Saving mesh to file.\n";
//
//	std::filebuf fb;
//	fb.open(outputFile, std::ios::out);
//	std::ostream os(&fb);
//
//	os << "ply\n"
//	   << "format ascii 1.0\n"
//	   << "element vertex " << pointCount << "\n"
//	   << "property float x\n"
//	   << "property float y\n"
//	   << "property float z\n"
//	   << "element face " << numberOfTriangles << "\n"
//	   << "property list uchar int vertex_index\n"
//	   << "end_header\n";
//
//	for (size_t i = 0; i < pointCount; ++i){
//		os << simplifiedPoints[i].x() << " " << simplifiedPoints[i].y() << " " << simplifiedPoints[i].z() << std::endl;
//	}
//
//	for (CDT::Face_iterator face = cdt.faces_begin(); face != cdt.faces_end(); ++face) {
//		os << 3 << " ";
//
//		if (flipFaces){
//			os << face->vertex(2)->info() << " " << face->vertex(1)->info() << " " << face->vertex(0)->info() << std::endl;
//		}else{
//			os << face->vertex(0)->info() << " " << face->vertex(1)->info() << " " << face->vertex(2)->info() << std::endl;
//		}
//	}
//
//	fb.close();

	log << "Successfully wrote mesh to: " << outputFile << "\n";
}

void Odm25dMeshing::printHelp() {
	bool printInCoutPop = log.isPrintingInCout();
	log.setIsPrintingInCout(true);

	log << "Usage: odm_25dmeshing -inputFile [plyFile] [optional-parameters]\n";
	log << "Create a 2.5D mesh from an oriented, classified point cloud (points with normals, classification and heightaboveground property) using a constrained delaunay triangulation. "
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



