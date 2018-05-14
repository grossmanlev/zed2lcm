#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <mutex>
#include <thread>
#include <ctime>
#include <string>
#include <chrono>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/obj_io.h>

#include <lcm/lcm-cpp.hpp>

#include "bot_core/pointcloud_t.hpp"

#include <nlohmann/json.hpp>
#include "drake/lcmt_viewer2_comms.hpp"

using json = nlohmann::json;

// std::mutex mesh_mutex;
// pcl::PolygonMesh mesh;

int global_count = 0;

bool process_pc = 0;

lcm::LCM lcm_sub;

class Handler 
{
    public:
        ~Handler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const bot_core::pointcloud_t* msg)
        {
        	printf("Received message on channel \"%s\":\n", chan.c_str());

        	auto t = std::chrono::system_clock::now();

        	std::cout << msg->seq << " @ " << std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count() - msg->utime << std::endl;

        	if (process_pc == 1) {
        		process_pc = 0;
        		return;
        	}
        	process_pc++;


			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

			cloud->width = msg->n_points;
			cloud->height = 1;
			cloud->points.resize(cloud->width * cloud->height);
			int good_points = 0;
			for(int i = 0; i < msg->n_points; i++) {
				if(!std::isfinite(msg->points[i][0]) ||
					!std::isfinite(msg->points[i][1]) ||
					!std::isfinite(msg->points[i][2]) ||
					(msg->points[i][0] == 0 && msg->points[i][1] == 0 && msg->points[i][2] == 0))
					continue;
				else {
					good_points++;
				}
			}
			cloud->width = good_points;
			cloud->points.resize(cloud->width * cloud->height);
			int count = 0;
			for(int i = 0; i < msg->n_points; i++) {
				if(!std::isfinite(msg->points[i][0]) ||
					!std::isfinite(msg->points[i][1]) ||
					!std::isfinite(msg->points[i][2]) ||
					(msg->points[i][0] == 0 && msg->points[i][1] == 0 && msg->points[i][2] == 0))
					continue;
				else {
					cloud->points[count].x = msg->points[i][0];
					cloud->points[count].y = msg->points[i][1];
					cloud->points[count].z = msg->points[i][2];
					count++;
				}
			}

			// pcl::io::savePCDFileASCII ("mesh.pcd", *cloud);
  	// 		pcl::PCLPointCloud2 cloud_blob;
  	// 		pcl::io::loadPCDFile ("mesh.pcd", cloud_blob);
  	// 		pcl::fromPCLPointCloud2 (cloud_blob, *cloud);

			// Statistical Outlier Removal Filter
			// pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			// sor.setInputCloud (cloud);
			// sor.setMeanK (10);
			// sor.setStddevMulThresh (2.0);
			// sor.filter (*cloud_filtered);

			// std::vector<int> indices;
			// pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, indices);

			// if(global_count % 10 == 0) {
			// 	pcl::PCDWriter writer;
			// 	writer.write<pcl::PointXYZ> ("cloud.pcd", *cloud, false);
			// 	writer.write<pcl::PointXYZ> ("cloud_filtered.pcd", *cloud_filtered, false);
			// 	printf("Pointclouds written!\n");
			// }

			pcl::VoxelGrid<pcl::PointXYZ> vox;
			vox.setInputCloud (cloud);
			vox.setLeafSize (0.05f, 0.05f, 0.05f);
			vox.filter (*cloud_filtered);


			//Estimating Normals
			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
			pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
			tree->setInputCloud (cloud_filtered);
			n.setInputCloud (cloud_filtered);
			n.setSearchMethod (tree);
			n.setKSearch(20);
			//n.setRadiusSearch(0.5);
			n.compute (*normals);

			// // Concatenate XYZ and normals
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
			pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);

			// // Create search tree*
			pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
			tree2->setInputCloud (cloud_with_normals);

			pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
			pcl::PolygonMesh triangles;

			// // Set the maximum distance between connected points (maximum edge length)
			gp3.setSearchRadius (0.25);

			// // Set typical values for the parameters
			gp3.setMu (2.5);
			gp3.setMaximumNearestNeighbors (100);
			gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
			gp3.setMinimumAngle(M_PI/18); // 10 degrees
			gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
			gp3.setNormalConsistency(false);

			// // Get result
			gp3.setInputCloud (cloud_with_normals);
			gp3.setSearchMethod (tree2);
			gp3.reconstruct (triangles);

			json j_mesh;
			j_mesh["type"] = "mesh_data";
			j_mesh["vertices"] = json::array();
			j_mesh["faces"] = json::array();

			// NOTE: may need to make faces 1-indexed

			bool print = true;
			for (auto i : cloud_filtered->points) {
				std::vector<float> tmp {i.x, i.y, i.z};
				j_mesh["vertices"].push_back(tmp);
			}
			for (auto i : triangles.polygons) {
				j_mesh["faces"].push_back(i.vertices);
			}

			std::cout << "Verts: " << j_mesh["vertices"].size() <<std::endl;
			std::cout << "Faces: " << j_mesh["faces"].size() <<std::endl;

			json data_geometry;
			data_geometry["path"] = {"robot1", "link1"};
			data_geometry["geometry"] = j_mesh;

			json payload;
			payload["timestamp"] = (int)time(0);
			payload["setgeometry"] = {data_geometry};
			payload["settransform"] = json::array();
			payload["delete"] = json::array();

			std::string payload_str = payload.dump();
			std::vector<unsigned char> payload_vector(payload_str.begin(), payload_str.end());

			drake::lcmt_viewer2_comms msg2;
			msg2.format = "treeviewer_json";
			msg2.format_version_major = 1;
			msg2.format_version_minor = 0;
			msg2.data = payload_vector;
			msg2.num_bytes = payload_str.length();
			lcm_sub.publish("DIRECTOR_TREE_VIEWER_REQUEST_<11>", &msg2);


			auto end = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_secs = end - t;
			std::cout << "Processing time (secs): " << elapsed_secs.count() << std::endl;


			//pcl::io::saveOBJFile ("tmp.obj", triangles);
			//rename("tmp.obj", "/home/lev/dev/test/mesh.obj");


		    // if(mesh_mutex.try_lock()) {
		    // 	mesh = triangles;
		    // 	mesh_mutex.unlock();
		    // }

			//printf("Points: %d\n", msg->n_points);
			//usleep(100000);


			//printf("%d\n", global_count);
			global_count++;
			
        }
};

// void visualize() {
// 	pcl::io::loadOBJFile("mesh.obj",mesh);
// 	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
// 	viewer->setBackgroundColor (0, 0, 0);
// 	viewer->addPolygonMesh(mesh,"mesh");
// 	viewer->addCoordinateSystem (1.0);
// 	viewer->initCameraParameters ();
// 	while (!viewer->wasStopped ()){
// 	    viewer->spinOnce (100);
// 	    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
// 	    if(mesh_mutex.try_lock()) {
// 	    	viewer->updatePolygonMesh(mesh, "mesh");
// 	    	mesh_mutex.unlock();
// 	    }
// 	}
// }

int main (int argc, char const* argv[])
{
	if(!lcm_sub.good())
		return 1;
	
	Handler handlerObject;
	lcm_sub.subscribe("DRAKE_POINTCLOUD_TEST", &Handler::handleMessage, &handlerObject);
	
	//std::thread workerThread(visualize);

	while(0 == lcm_sub.handle());

	//workerThread.join(); 

	return 0;
}
