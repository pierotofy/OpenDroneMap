#include "Odm25dMeshing.hpp"

//We define a vertex_base with info. The "info" (size_t) allow us to keep track of the original point index.
typedef CGAL::Triangulation_vertex_base_with_info_2<size_t, Kernel> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
typedef CGAL::Delaunay_triangulation_2<Kernel, Tds> DT;
typedef DT::Point cgalPoint;
typedef DT::Vertex_circulator Vertex_circulator;

// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif

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

		buildMesh();

		log << "Done!" << "\n";

		// TODO

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
		} else if (argument == "-verbose") {
			log.setIsPrintingInCout(true);
		} else if (argument == "-maxVertexCount" && argIndex < argc) {
//            ++argIndex;
//            if (argIndex >= argc)
//            {
//                throw OdmMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
//            }
//            std::stringstream ss(argv[argIndex]);
//            ss >> maxVertexCount_;
//            if (ss.bad())
//            {
//                throw OdmMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
//            }
//            log << "Vertex count was manually set to: " << maxVertexCount_ << "\n";
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
	  PlyInterpreter interpreter(points);

	  std::ifstream in(inputFile);
	  if (!in || !CGAL::read_ply_custom_points (in, interpreter, Kernel())){
		  throw Odm25dMeshingException(
		  				"Error when reading points and normals from:\n" + inputFile + "\n");
	  }

	  log << "Successfully loaded " << points.size() << " points from file\n";
}

void Odm25dMeshing::buildMesh(){
	size_t pointCountBeforeOutRemoval = points.size();

	log << "Removing outliers\n";

	const unsigned int nb_neighbors = 24;
	const double removed_percentage = 5;

	points.erase(CGAL::remove_outliers(points.begin(), points.end(),
			CGAL::First_of_pair_property_map<Pwc>(), nb_neighbors, removed_percentage),
			points.end());
	std::vector<Pwc>(points).swap(points);

	size_t pointCount = points.size();

	log << "Removed " << (pointCountBeforeOutRemoval - pointCount) << " points\n";

	if (pointCount < 3){
		throw Odm25dMeshingException("Not enough points");
	}

	log << "Smoothing point set\n";

	// jet_smooth_point_set does not seem to support pairs, so we need
	// to copy the points over
	std::vector<Point3> smoothPoints;
	for (size_t i = 0; i < pointCount; i++){
		smoothPoints.push_back(points[i].first);
	}
	CGAL::jet_smooth_point_set<Concurrency_tag>(smoothPoints.begin(), smoothPoints.end(), nb_neighbors);

	log << "Smoothing completed\n";

	std::vector< std::pair<cgalPoint, size_t > > pts;
	try{
		pts.reserve(pointCount);
	} catch (const std::bad_alloc&){
		throw Odm25dMeshingException("Not enough memory");
	}

	for (size_t i = 0; i < pointCount; ++i){
		pts.push_back(std::make_pair(cgalPoint(smoothPoints[i].x(), smoothPoints[i].y()), i));
	}

	log << "Computing delaunay triangulation\n";

	//The delaunay triangulation is built according to the 2D point cloud
	DT dt(pts.begin(), pts.end());

	unsigned int numberOfTriangles = static_cast<unsigned >(dt.number_of_faces());
	unsigned int triIndexes = dt.number_of_faces()*3;

	if (numberOfTriangles == 0) throw Odm25dMeshingException("No triangles in resulting mesh");

	log << "Computed " << numberOfTriangles << " triangles\n";

	// Convert to tinyply format
	std::vector<float> vertices;
	std::vector<uint8_t> colors;
	std::vector<int> vertexIndicies;

	try{
		vertices.reserve(pointCount);
		colors.reserve(pointCount);
		vertexIndicies.reserve(triIndexes);
	} catch (const std::bad_alloc&){
		throw Odm25dMeshingException("Not enough memory");
	}

	for (size_t i = 0; i < pointCount; ++i){
		vertices.push_back(smoothPoints[i].x());
		vertices.push_back(smoothPoints[i].y());
		vertices.push_back(smoothPoints[i].z());

		colors.push_back(points[i].second[0]);
		colors.push_back(points[i].second[1]);
		colors.push_back(points[i].second[2]);
	}

	for (DT::Face_iterator face = dt.faces_begin(); face != dt.faces_end(); ++face) {
		vertexIndicies.push_back(static_cast<int>(face->vertex(0)->info()));
		vertexIndicies.push_back(static_cast<int>(face->vertex(1)->info()));
		vertexIndicies.push_back(static_cast<int>(face->vertex(2)->info()));
	}

	// TODO: apply fairing algo http://doc.cgal.org/latest/Polygon_mesh_processing/group__PMP__meshing__grp.html#gaa091c8368920920eed87784107d68ecf

	log << "Removing spikes\n";

	const float THRESHOLD = 0.1;
	const int PASSES = 20;

	// TODO: check these...

	std::vector<float> heights;
	unsigned int spikesRemoved = 0;
	float current_threshold = THRESHOLD;

	for (int i = 1; i <= PASSES; i++){
		log << "Pass " << i << " of " << PASSES << "\n";

	for (DT::Vertex_iterator vertex = dt.vertices_begin(); vertex != dt.vertices_end(); ++vertex){
		// Check if the height between this vertex and its
		// incident vertices is greater than THRESHOLD
		Vertex_circulator vc = dt.incident_vertices(vertex), done(vc);

		if (vc != 0){
			float height = vertices[vertex->info() * 3 + 2];
			bool spike = false;

			do{
				if (dt.is_infinite(vc)) continue;

				float ivHeight = vertices[vc->info() * 3 + 2];
				if (fabs(height - ivHeight) > current_threshold) spike = true;

				heights.push_back(ivHeight);
			}while(++vc != done);

			if (spike){
				// Replace the height of the vertex by the median height
				// of its incident vertices
				std::sort(heights.begin(), heights.end());

//				points[vertex->info()].first = Point3(points[vertex->info()].first.x(),
//													points[vertex->info()].first.y(),
//													heights[heights.size() / 2]);

				vertices[vertex->info() * 3 + 2] = heights[heights.size() / 2];
//
//				colors[vertex->info() * 3] = 255;
//				colors[vertex->info() * 3 + 1] = 0;
//				colors[vertex->info() * 3 + 2] = 0;


				spikesRemoved++;
			}

			heights.clear();
		}
	}

		current_threshold /= 2;
	}

	log << "Saving mesh to file.\n";

	std::filebuf fb;
	fb.open(outputFile, std::ios::out | std::ios::binary);
	std::ostream outputStream(&fb);

	tinyply::PlyFile plyFile;
	plyFile.add_properties_to_element("vertex", {"x", "y", "z"}, vertices);
	plyFile.add_properties_to_element("vertex", { "diffuse_red", "diffuse_green", "diffuse_blue"}, colors);
	plyFile.add_properties_to_element("face", { "vertex_index" }, vertexIndicies, 3, tinyply::PlyProperty::Type::INT8);

	plyFile.write(outputStream, false);
	fb.close();

	log << "Successfully wrote mesh to:\n" << outputFile << "\n";
}

void Odm25dMeshing::printHelp() {
	bool printInCoutPop = log.isPrintingInCout();
	log.setIsPrintingInCout(true);

	log << "odm_25dmeshing\n\n";

	log << "Purpose:" << "\n";
	log
			<< "Create a 2.5D mesh from an oriented point cloud (points with normals) using a constrained delaunay triangulation"
			<< "\n";

	log << "Usage:" << "\n";
	log	<< "The program requires a path to an input PLY point cloud file, all other input parameters are optional."
			<< "\n\n";

	log << "The following flags are available\n";
	log	<< "Call the program with flag \"-help\", or without parameters to print this message, or check any generated log file.\n";
	log
			<< "Call the program with flag \"-verbose\", to print log messages in the standard output stream as well as in the log file.\n\n";

	log	<< "Parameters are specified as: \"-<argument name> <argument>\", (without <>), and the following parameters are configureable: "
			<< "\n";
	log << "\"-inputFile <path>\" (mandatory)" << "\n";
	log	<< "\"Input ply file that must contain a point cloud with normals.\n\n";

	log << "\"-outputFile <path>\" (optional, default: odm_mesh.ply)" << "\n";
	log << "\"Target file in which the mesh is saved.\n\n";

	log << "\"-logFile <path>\" (optional, default: odm_25dmeshing_log.txt)"
			<< "\n";

	log.setIsPrintingInCout(printInCoutPop);
}



