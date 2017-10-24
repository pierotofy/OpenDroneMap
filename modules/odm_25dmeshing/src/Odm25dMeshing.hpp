#pragma once

// STL
#include <string>
#include <iostream>
#include <unordered_map>
#include <cstring>
#include <algorithm>

#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/impl/region_growing.hpp>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>

#include "Logger.hpp"

class Odm25dMeshing {
public:
	Odm25dMeshing() :
			log(false),
			meshPoints( new pcl::PointCloud<pcl::PointNormal> ),
			nongroundPoints( new pcl::PointCloud<pcl::PointNormal> ),
		    meshCreator(pcl::Poisson<pcl::PointNormal>::Ptr(new pcl::Poisson<pcl::PointNormal>())),
		    mesh(pcl::PolygonMeshPtr(new pcl::PolygonMesh)),
		    decimatedMesh(pcl::PolygonMeshPtr(new pcl::PolygonMesh))
			{};

	~Odm25dMeshing() {};

	/*!
	 * \brief   run     Runs the meshing functionality using the provided input arguments.
	 *                  For a list of accepted arguments, please see the main page documentation or
	 *                  call the program with parameter "-help".
	 * \param   argc    Application argument count.
	 * \param   argv    Argument values.
	 * \return  0       If successful.
	 */
	int run(int argc, char **argv);

private:

	/*!
	 * \brief parseArguments    Parses command line arguments.
	 * \param   argc    Application argument count.
	 * \param   argv    Argument values.
	 */
	void parseArguments(int argc, char** argv);

	/*!
	 * \brief loadPointCloud    Loads a PLY file with points and normals from file.
	 */
	void loadPointCloud();
	void createMesh();
	void mergePlanarPoints(pcl::PointCloud<pcl::PointNormal>::Ptr points);

    /*!
     * \brief writePlyFile  Writes the mesh to file on the Ply format.
     */
    void writePlyFile();

	/*!
	 * \brief printHelp     Prints help, explaining usage. Can be shown by calling the program with argument: "-help".
	 */
	void printHelp();

    /*!
     * \brief decimateMesh  Performs post-processing on the form of quadric decimation to generate a mesh
     *                      that has a higher density in areas with a lot of structure.
     */
    void decimateMesh();

    /*!
     * \brief calcTreeDepth Attepts to calculate the depth of the tree using the point cloud.
     *                      The function makes the assumption points are located roughly in a plane
     *                      (fairly reasonable for ortho-terrain photos) and tries to generate a mesh using
     *                      an octree with an appropriate resolution.
     * \param nPoints       The total number of points in the input point cloud.
     * \return              The calcualated octree depth.
     */
    int calcTreeDepth(size_t nPoints);

	Logger log;

	std::string inputFile = "";
	std::string outputFile = "odm_25dmesh.ply";
	std::string logFilePath = "odm_25dmeshing_log.txt";

    unsigned int maxVertexCount = 100000;  /**< Desired output vertex count. */
    unsigned int treeDepth = 0;    /**< Depth of octree used for reconstruction. */

    double samplesPerNode = 1.0;     /**< Samples per octree node.*/
    double solverDivide = 9.0;       /**< Depth at which the Laplacian equation solver is run during surface estimation.*/
    double decimationFactor = 0.0;   /**< Percentage of points to remove when decimating the mesh. */

	pcl::PointCloud<pcl::PointNormal>::Ptr meshPoints;
	pcl::PointCloud<pcl::PointNormal>::Ptr nongroundPoints;
//	pcl::PointCloud<pcl::PointNormal>::Ptr neargroundPoints;

    pcl::Poisson<pcl::PointNormal>::Ptr meshCreator;    /**< PCL poisson meshing class. */
    pcl::PolygonMeshPtr mesh;                      /**< PCL polygon mesh. */
    pcl::PolygonMeshPtr decimatedMesh;
};

class Odm25dMeshingException: public std::exception {

public:
	Odm25dMeshingException() :
			message("Error in Odm25dMeshing") {
	}
	Odm25dMeshingException(std::string msgInit) :
			message("Error in Odm25dMeshing:\n" + msgInit) {
	}
	~Odm25dMeshingException() throw () {
	}
	virtual const char* what() const throw () {
		return message.c_str();
	}

private:
	std::string message; /**< The error message **/
};
