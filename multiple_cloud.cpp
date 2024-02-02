/*#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/poisson.h>


#include <thread>
#include <chrono>
#include <string>
#include <pcl/registration/icp.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/io/ply_io.h>
*/
#include <iostream> 
#include <pcl/io/pcd_io.h> //Legge i files
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h> //Contiene Le varie tipologie di Punti (XYZ, RGB, ecc)
#include <pcl/features/normal_3d.h> 
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/surface/concave_hull.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

int main (int argc, char * argv[]) {

	if (argc == 1) { 
		return -1;
	}

	//INPUT FILE
	fs::path pc_dir ("/home/PointCloud");
	fs::path pc_file (argv[1]);
	fs::path input_path = pc_dir / pc_file;

	//OUTPUT FILE
	fs::path mesh_dir ("/home/Mesh");
	fs::path mesh_file (argv[2]);
	fs::path output_path = mesh_dir / mesh_file;

    //READ POINT CLOUD
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);   
    pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(input_path.string(), *cloud);
    std::cout << "Grandezza PointCloud: " <<cloud->width * cloud->height << std::endl;

    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree->setInputCloud(cloud);

    
    pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh);


    // ### FUNZIONA MA NON RIMANGONO I VETTORI NORMALI ###
    /*
    pcl::ConcaveHull<pcl::PointXYZRGBNormal> concave_hull;
    concave_hull.setInputCloud(cloud);
    concave_hull.setAlpha(0.5);
    concave_hull.reconstruct(*mesh_ptr);
	*/

    // ### FUNZIONA E RIMANGONO I VETTORI NORMALI ###
    
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

	pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
	tree2->setInputCloud(cloud);
	gp3.setSearchRadius(0.1);
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors(50);
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(true);
	gp3.setConsistentVertexOrdering(true);
	gp3.setSearchMethod(tree);
	gp3.setInputCloud(cloud);
	gp3.reconstruct (*mesh_ptr);
	
    
    /*
    pcl::MarchingCubesHoppe<pcl::PointXYZRGBNormal> mch;

	mch.setInputCloud(cloud);
	mch.reconstruct(*mesh_ptr);
	*/

    //std::cout << "Mesh vertices? " << mesh_ptr->cloud.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::fromPCLPointCloud2(mesh_ptr->cloud, *vertices);


    std::cout << "Totali vertici nella Mesh: " << vertices->size() << std::endl;

	int K = 1; 
    for(int i = 0; i < vertices->size(); i++)
    {
    	pcl::PointXYZRGBNormal query_point = vertices->points[i]; //Vertice della mesh

    	// Number of nearest neighbors to search for
	    std::vector<int> indices(K);
	    std::vector<float> distances(K);
	    
	    tree->nearestKSearch(query_point, K, indices, distances);

	    vertices->points[i].r = cloud->points[indices[0]].r;
	    vertices->points[i].g = cloud->points[indices[0]].g;
	    vertices->points[i].b = cloud->points[indices[0]].b;

    }

    pcl::PCLPointCloud2 vertices_with_colors;
    pcl::toPCLPointCloud2(*vertices, vertices_with_colors);

    mesh_ptr->cloud = vertices_with_colors;

    //SAVE MESH TO PLY FILE
    pcl::io::savePLYFile(output_path.string(), *mesh_ptr);
    std::cout << "file: " << output_path.string() << " salvato!" << std::endl;




    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D visualizer"));
    // Visualizza la nuvola di punti in bianco
    viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "cloud");

    // Visualizza l'involucro concavo in rosso
    //viewer.addPointCloud(hull, "hull");
    //viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "hull");

    // Esegui il visualizzatore
    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }


    /*
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);
    ne.setInputCloud (cloud);
    ne.setSearchMethod (tree);
    ne.setKSearch (20); // Use 20 nearest neighbors to estimate normals
    //ne.setRadiusSearch (1.0f);
    ne.compute (*normals);


    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);


    pcl::io::savePCDFileASCII ("/home/rosbags/cmagazzino_with_normals.pcd", *cloud_with_normals);
    std::cerr << "Saved " << cloud_with_normals->points.size () << " data points with normals to: pointCloud_with_normals.pcd" << std::endl;
    */

	//1375119585.678038463.pcd  1375119585.778108330.pcd

	//1375119536.115010412.pcd 1375119536.147135664.pcd 1375119542.147581697.pcd
	/*
	std::string files[24] =
	{ "0.pcd",  "10.pcd","12.pcd","14.pcd","16.pcd","18.pcd","2.pcd","21.pcd","23.pcd","4.pcd","6.pcd","8.pcd","1.pcd","11.pcd","13.pcd","15.pcd","17.pcd","19.pcd","20.pcd","22.pcd","3.pcd","5.pcd","7.pcd","9.pcd"};

	pcl::PointCloud<pcl::PointXYZ>::Ptr global_point_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	 for (int i = 0; i < 24; i++)
	 {
	 		std::cout << files[i] << "\n";
	 		pcl::PCLPointCloud2 temp;
	 		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	 		std::string path = "/home/translated_clouds/" + files[i];
	 		pcl::io::loadPCDFile (path, temp);
	 		pcl::fromPCLPointCloud2 (temp, *temp_cloud);

	 		//*global_point_cloud += *temp_cloud;
	 }
    */

	/*
	//Leggo La point Cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2 first_cloud_blob;
    pcl::io::loadPCDFile ("/home/translated_clouds/0.pcd", first_cloud_blob);
    pcl::fromPCLPointCloud2 (first_cloud_blob, *first_cloud);
	*/

}

/*


	//Filtro la Point Cloud per ridurre il numero di punti
	//In questo caso utilizzo un filtro chiamato Voxel Grid
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> gridFilter;
  gridFilter.setInputCloud (cloud);
  gridFilter.setLeafSize (0.02f, 0.02f, 0.02f);
  gridFilter.filter (*cloud_filtered);

	//Eseguo uno Smoothing della point cloud utilizzando Moving Least Square
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>); //KD-Tree

	tree->setInputCloud(cloud_filtered);
	//Variabile per le normali calcolate da MLS
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_normals (new pcl::PointCloud<pcl::PointNormal> ());


	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setComputeNormals (true);
	mls.setPolynomialOrder (2);
	mls.setSearchRadius (0.1);
	mls.setSearchMethod (tree);
	mls.setInputCloud (cloud_filtered);

	mls.process (*mls_normals);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree;
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_normals (new pcl::PointCloud<pcl::PointNormal> ());
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	mls.setInputCloud (cloud_filtered);
	mls.setComputeNormals(true);
	mls.setPolynomialOrder(2);
	//mls.setSearchMethod (tree);
	mls.setSearchRadius(0.1);

  mls.process (*mls_normals);


  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh mesh;
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(mls_normals);

  gp3.setSearchRadius(0.1);
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors(50);
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(true);
  gp3.setConsistentVertexOrdering(true);

  gp3.setSearchMethod(tree2);
  gp3.setInputCloud(mls_normals);
  gp3.reconstruct (mesh);


	pcl::MarchingCubesHoppe<pcl::PointNormal> mch;
	pcl::PolygonMesh mesh;

	mch.setInputCloud(mls_normals);
	mch.reconstruct(mesh);


	pcl::Poisson<pcl::PointNormal> poisson;
	pcl::PolygonMesh mesh;

	poisson.setDepth (4);
	poisson.setSolverDivide (4);
	poisson.setIsoDivide (4);
  poisson.setPointWeight (2.0f);
  poisson.setInputCloud (mls_normals);
  poisson.reconstruct(mesh);

*/
