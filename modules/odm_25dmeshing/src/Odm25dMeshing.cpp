#include "Odm25dMeshing.hpp"

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

		mergeAndSmoothPlanarPoints(nongroundPoints);

		createMesh();

        decimateMesh();

        writePlyFile();

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
		} else if(argument == "-maxVertexCount" && argIndex < argc){
			++argIndex;
			if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
			std::stringstream ss(argv[argIndex]);
			ss >> maxVertexCount;
			if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
			log << "Vertex count was manually set to: " << maxVertexCount << "\n";
		}else if(argument == "-octreeDepth" && argIndex < argc){
			++argIndex;
			if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
			std::stringstream ss(argv[argIndex]);
			ss >> treeDepth;
			if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
			log << "Octree depth was manually set to: " << treeDepth << "\n";
		} else if(argument == "-solverDivide" && argIndex < argc){
			++argIndex;
			if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
			std::stringstream ss(argv[argIndex]);
			ss >> solverDivide;
			if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
			log << "Numerical solver divisions was manually set to: " << solverDivide << "\n";
		} else if(argument == "-samplesPerNode" && argIndex < argc){
			++argIndex;
			if (argIndex >= argc) throw Odm25dMeshingException("Argument '" + argument + "' expects 1 more input following it, but no more inputs were provided.");
			std::stringstream ss(argv[argIndex]);
			ss >> samplesPerNode;
			if (ss.bad()) throw Odm25dMeshingException("Argument '" + argument + "' has a bad value (wrong type).");
			log << "The number of samples per octree node was manually set to: " << samplesPerNode << "\n";
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
						"Argument '" + argument
								+ "' has a bad value. (file not accessible)");
			}
			testFile.close();
			log << "Reading point cloud at: " << inputFile << "\n";
		} else if (argument == "-outputFile" && argIndex < argc) {
			++argIndex;
			if (argIndex >= argc) {
				throw Odm25dMeshingException(
						"Argument '" + argument
								+ "' expects 1 more input following it, but no more inputs were provided.");
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
			throw Odm25dMeshingException(
					"Unrecognised argument '" + argument + "'");
		}
	}
}

void Odm25dMeshing::loadPointCloud() {
	const char CLASS_GROUND = 2;
	const float HAG_THRESHOLD = 1.0; // 1 meters

	pcl::PCLPointCloud2 blob;

	log << "Loading point cloud... ";

	if (pcl::io::loadPLYFile(inputFile.c_str(), blob) == -1) {
		throw Odm25dMeshingException("Error when reading from: " + inputFile);
	}

	log << "OK\n";

	log << "Scanning fields... ";

	pcl::PCLPointField *posX = NULL, *posY = NULL, *posZ = NULL,
			*normalX = NULL, *normalY = NULL, *normalZ = NULL, *classification = NULL,
			*hag = NULL;

#define ASSIGN(_name, _field) if (blob.fields[i].name == _name){ _field = &blob.fields[i]; log << _name << " "; continue; }

	for (size_t i = 0; i < blob.fields.size(); ++i) {
		ASSIGN("x", posX);
		ASSIGN("y", posY);
		ASSIGN("z", posZ);
		ASSIGN("normal_x", normalX);
		ASSIGN("normal_y", normalY);
		ASSIGN("normal_z", normalZ);
		ASSIGN("nx", normalX);
		ASSIGN("ny", normalY);
		ASSIGN("nz", normalZ);
		ASSIGN("classification", classification);
		ASSIGN("heightaboveground", hag);
	}

	log << "OK\n";

	if (posX == NULL || posY == NULL || posZ == NULL)
		throw Odm25dMeshingException(
				"Position attributes (x,y,z) missing from input");
	if (normalX == NULL || normalY == NULL || normalZ == NULL)
		throw Odm25dMeshingException(
				"Normal attributes (normal_x,normal_y,normal_z) missing from input");
	if (posX->datatype != pcl::PCLPointField::FLOAT32
			&& posX->datatype != pcl::PCLPointField::FLOAT64)
		throw Odm25dMeshingException(
				"Only float and float64 types are supported for position information");
	if (normalX->datatype != pcl::PCLPointField::FLOAT32
			&& normalX->datatype != pcl::PCLPointField::FLOAT64)
		throw Odm25dMeshingException(
				"Only float and float64 types are supported for normal information");

	if (classification->datatype != pcl::PCLPointField::UINT8) classification = NULL;
	if (classification == NULL) log << "WARNING: Classification attribute missing. Will treat all points as ground.\n";
	if (hag == NULL) log << "WARNING: heightaboveground attribute missing. Resulting mesh might have more artifacts.\n";

	uint8_t pointClass = 2;
	float pointHag = std::numeric_limits<float>::min();

	pcl::PointCloud<pcl::PointNormal> allPoints;
	allPoints.reserve(blob.width * blob.height);

	for (size_t point_step = 0, i = 0; point_step < blob.data.size();
			point_step += blob.point_step, i++) {
		uint8_t *point = blob.data.data() + point_step;

		if (posX->datatype == pcl::PCLPointField::FLOAT64) {
			allPoints[i].x =
					*(reinterpret_cast<double *>(point + posX->offset));
			allPoints[i].y =
					*(reinterpret_cast<double *>(point + posY->offset));
			allPoints[i].z =
					*(reinterpret_cast<double *>(point + posZ->offset));
		} else if (posX->datatype == pcl::PCLPointField::FLOAT32) {
			allPoints[i].x = *(reinterpret_cast<float *>(point + posX->offset));
			allPoints[i].y = *(reinterpret_cast<float *>(point + posY->offset));
			allPoints[i].z = *(reinterpret_cast<float *>(point + posZ->offset));
		}

		if (normalX->datatype == pcl::PCLPointField::FLOAT64) {
			allPoints[i].normal_x = *(reinterpret_cast<double *>(point
					+ normalX->offset));
			allPoints[i].normal_y = *(reinterpret_cast<double *>(point
					+ normalY->offset));
			allPoints[i].normal_z = *(reinterpret_cast<double *>(point
					+ normalZ->offset));
		} else if (normalX->datatype == pcl::PCLPointField::FLOAT32) {
			allPoints[i].normal_x = *(reinterpret_cast<float *>(point
					+ normalX->offset));
			allPoints[i].normal_y = *(reinterpret_cast<float *>(point
					+ normalY->offset));
			allPoints[i].normal_z = *(reinterpret_cast<float *>(point
					+ normalZ->offset));
		}

		if (classification != NULL) {
			pointClass = *(reinterpret_cast<uint8_t *>(point
					+ classification->offset));
		}

		if (hag != NULL) {
			if (hag->datatype == pcl::PCLPointField::FLOAT64)
				pointHag = *(reinterpret_cast<double *>(point + hag->offset));
			if (hag->datatype == pcl::PCLPointField::FLOAT32)
				pointHag = *(reinterpret_cast<float *>(point + hag->offset));
		}

		if (pointClass == CLASS_GROUND) {
			meshPoints->push_back(allPoints[i]);
		} else {
			if (pointHag >= HAG_THRESHOLD) {
				nongroundPoints->push_back(allPoints[i]);
			}else{
				neargroundPoints->push_back(allPoints[i]);
			}
		}
	}

	log << "Loaded " << meshPoints->size() << " points for meshing\n";
	log << "Loaded " << nongroundPoints->size() << " non ground points\n";
	log << "Loaded " << neargroundPoints->size() << " near ground points\n";
}

// Merges ground with nonground points
// while attempting to detect and smooth planar surfaces (ex. buildings)
// and optionally discarding non planar surfaces (ex. trees)
void Odm25dMeshing::mergeAndSmoothPlanarPoints(pcl::PointCloud<pcl::PointNormal>::Ptr points) {
	if (points->size() < 3) return;

	const int K = 30,
		  MIN_CLUSTER_SIZE = 100;
	const float SURFACE_RATIO_THRESHOLD = 0.2f;

	log << "Extracting clusters... ";

	pcl::search::Search<pcl::PointNormal>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointNormal> > (new pcl::search::KdTree<pcl::PointNormal>);
	pcl::RegionGrowing<pcl::PointNormal, pcl::PointNormal> reg;
	reg.setMinClusterSize (MIN_CLUSTER_SIZE);
	reg.setMaxClusterSize (points->size());
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (K);
	reg.setInputCloud (points);
	reg.setInputNormals (points);

	reg.setSmoothnessThreshold (45 / 180.0 * M_PI);
	reg.setCurvatureThreshold (3);


	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	log << " found " << clusters.size() << " clusters\n";
	log << "Detecting per segment surfaces... \n";

	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	for (size_t cluster_idx = 0; cluster_idx < clusters.size(); cluster_idx++){
		pcl::PointCloud<pcl::PointNormal>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointNormal>(*points, clusters[cluster_idx].indices));
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
		tree->setInputCloud(cluster_cloud);

		size_t surface_points = 0;
		size_t points_in_cluster = cluster_cloud.get()->points.size();
		size_t surface_points_needed = points_in_cluster * SURFACE_RATIO_THRESHOLD + 1; // For shortcut

		for (auto point = cluster_cloud.get()->points.begin(); point != cluster_cloud.get()->points.end(); point++){
			size_t same_normal_direction_count = 0;

			// Find neighbors
			if (tree->nearestKSearch(*point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0){
				size_t num_points = pointIdxNKNSearch.size();
				for (size_t k = 0; k < num_points; k++){
					const pcl::PointNormal &p = cluster_cloud.get()->at(pointIdxNKNSearch[k]);

					if (point->getNormalVector3fMap().dot(p.getNormalVector3fMap()) > 0.9){
						same_normal_direction_count++;
					}
				}
			}

			if (same_normal_direction_count >= pointIdxNKNSearch.size() * 0.8f){
				surface_points++;

				if (surface_points >= surface_points_needed && surface_points >= MIN_CLUSTER_SIZE) break; // Simple optimization
			}
		}

		float surface_ratio = (float)surface_points / (float)points_in_cluster;
		log << "Segment #" << cluster_idx << " (points: " << points_in_cluster << ", plane points: " << surface_points << " (" << (surface_ratio * 100.0f) << "%)\n";

		// At least SURFACE_RATIO_THRESHOLD% of points
		// in this segment are probably a man-made structure
		if (surface_points >= surface_points_needed && surface_points >= MIN_CLUSTER_SIZE){
			pcl::MovingLeastSquares<pcl::PointNormal, pcl::PointNormal> mls;
			pcl::PointCloud<pcl::PointNormal> mls_points;

			mls.setInputCloud(cluster_cloud);
			mls.setPolynomialFit (true);
			mls.setSearchMethod (tree);
			mls.setSearchRadius (0.2);
			mls.process (mls_points);

			(*meshPoints) += mls_points;
		}else{
			// Tree, or something else

			// Do nothing
		}
	}
}

void Odm25dMeshing::createMesh() {
    // Attempt to calculate the depth of the tree if unspecified
    if (treeDepth == 0) treeDepth = calcTreeDepth(meshPoints->size());

    log << "Octree depth used for reconstruction is: " << treeDepth << "\n";
    log << "Estimated initial vertex count: " << pow(4, treeDepth) << "\n";

    meshCreator->setDepth(treeDepth);
    meshCreator->setSamplesPerNode(samplesPerNode);
    meshCreator->setInputCloud(meshPoints);

    // Guarantee manifold mesh.
    meshCreator->setManifold(true);

    // Begin reconstruction
    meshCreator->reconstruct(*mesh.get());

    log << "Reconstruction complete:\n";
    log << "Vertex count: " << mesh->cloud.width * mesh->cloud.height << "\n";
    log << "Triangle count: " << mesh->polygons.size() << "\n\n";

}

void Odm25dMeshing::decimateMesh(){
    if (maxVertexCount <= 0){
        log << "Vertex count not specified, decimation cancelled.\n";
        return;
    }

    if (maxVertexCount > mesh->cloud.height*mesh->cloud.width)
    {
        log << "Vertex count in mesh lower than initially generated mesh, unable to decimate.\n";
        return;
    }
    else
    {
        double reductionFactor = 1.0 - double(maxVertexCount)/double(mesh->cloud.height*mesh->cloud.width);
        log << "Decimating mesh, removing " << reductionFactor*100 << " percent of vertices.\n";

        pcl::MeshQuadricDecimationVTK decimator;
        decimator.setInputMesh(mesh);
        decimator.setTargetReductionFactor(reductionFactor);
        decimator.process(*decimatedMesh.get());

        log << "Decimation complete.\n";
        log << "Decimated vertex count: " << decimatedMesh->cloud.width << "\n";
        log << "Decimated triangle count: " << decimatedMesh->polygons.size() << "\n\n";

        mesh = decimatedMesh;
    }
}


int Odm25dMeshing::calcTreeDepth(size_t nPoints){
    // Assume points are located (roughly) in a plane.
    double squareSide = std::sqrt((double)nPoints);

    // Calculate octree depth such that if points were equally distributed in
    // a quadratic plane, there would be at least 1 point per octree node.
    int depth = 0;
    while(std::pow<double>(2,depth) < squareSide/2)
    {
        depth++;
    }
    return depth;
}

void Odm25dMeshing::printHelp() {
	bool printInCoutPop = log.isPrintingInCout();
	log.setIsPrintingInCout(true);

	log << "Usage: odm_25dmeshing -inputFile [plyFile] [optional-parameters]\n";
	log
			<< "Create a 2.5D mesh from an oriented, classified point cloud (points with normals, classification and heightaboveground property) using a constrained delaunay triangulation. "
			<< "The program requires a path to an input PLY point cloud file, all other input parameters are optional.\n\n";

	log << "	-inputFile	<path>	to PLY point cloud\n"
			<< "	-outputFile	<path>	where the output PLY 2.5D mesh should be saved (default: "
			<< outputFile << ")\n"
			<< "	-logFile	<path>	log file path (default: " << logFilePath
			<< ")\n" << "	-verbose	whether to print verbose output (default: "
			<< (printInCoutPop ? "true" : "false") << ")\n"
			<< "	-maxVertexCount	<0 - N>	Maximum number of vertices in the output mesh. The mesh might have fewer vertices, but will not exceed this limit. (default: "
			<< maxVertexCount << ")\n"

				// TODO!!

			<< "\n";

	log.setIsPrintingInCout(printInCoutPop);
}


void Odm25dMeshing::writePlyFile(){
    log << "Saving mesh to file.\n";

    if (pcl::io::savePLYFile(outputFile.c_str(), *mesh.get())  == -1) {
        throw Odm25dMeshingException("Error when saving mesh to file:\n" + outputFile + "\n");
    }else{
        log << "Successfully wrote mesh to: " << outputFile << "\n";
    }
}

