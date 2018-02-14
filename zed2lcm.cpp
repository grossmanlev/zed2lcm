#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <sl/Camera.hpp>
#include <lcm/lcm-cpp.hpp>

#include <vector>

#include "bot_core/pointcloud_t.hpp"
#include "bot_core/pointcloud2_t.hpp"

using namespace sl;


sl::Camera zed;
sl::Mat cloud;
lcm::LCM mlcm;
std::thread zed_callback;
int width, height;

void startZED();
void run();
void close();
bool quit;

int main (int argc, char** argv)
{
	if(!mlcm.good())
		return 1;
	
	bot_core::pointcloud2_t my_data;
	my_data.height = 640;
	my_data.width = 480;


	Camera zedl;
    InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_HD720;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = UNIT_METER;

    ERROR_CODE err = zed.open(init_params);
    if(err != SUCCESS) {
		zed.close();
        exit(-1);
	}

	width = (int) zed.getResolution().width / 2;
	height = (int) zed.getResolution().height / 2;

	// Start the camera
	startZED();



    //printf("Nothing happening...\n");

    //pcl::PointCloud<pcl::PointXYZ> cloud;

    // Fill in the cloud data
    //cloud.width    = 5;
    //cloud.height   = 1;
    //cloud.is_dense = false;
    //cloud.points.resize (cloud.width * cloud.height);

    //for (size_t i = 0; i < cloud.points.size (); ++i)
    //{
    //cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    //cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    //cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    //}

    //pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
    //std::cerr << "Saved " << cloud.points.size () << " data points to test_pcd.pcd." << std::endl;

    //for (size_t i = 0; i < cloud.points.size (); ++i)
    //std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
	
	while(true) {
		usleep(1000);
	}
    return (0);
}

void fromPCL(const pcl::PCLPointField &pcl_pf, bot_core::pointfield_t &pf)
{
	pf.name = pcl_pf.name;
	pf.offset = pcl_pf.offset;
	pf.datatype = pcl_pf.datatype;
	pf.count = pcl_pf.count;
}

//void fromPCL(const std::vector<pcl::PCLPointField> &pcl_pfs, std::vector<bot_core::pointfield_t> &pfs)
//{
//	pfs.resize(pcl_pfs.size());
//	std::vector<pcl::PCLPointField>::const_iterator it = pcl_pfs.begin();
//	int i = 0;
//	for(; it != pcl_pfs.end(); ++it, ++i) {
//		fromPCL(*(it), pfs[i]);
//	}
//}


void startZED() {
	quit = false;
	zed_callback = std::thread(run);
}

void run() {
	lcm::LCM new_lcm;
	if(!new_lcm.good())
		return;	
	while(!quit) {
		if(zed.grab() == SUCCESS) {
			zed.retrieveMeasure(cloud, MEASURE_XYZBGRA, MEM_CPU, width, height);
			pcl::PointCloud<pcl::PointXYZRGB> point_cloud;
			point_cloud.width = width;
			point_cloud.height = height;
			int size = width*height;
			point_cloud.points.resize(size);
			
			bot_core::pointcloud_t out;
			out.seq = 0;
			
			sl::Vector4<float>* cpu_cloud = cloud.getPtr<sl::float4>();
			for(int i = 0; i < size; i++) {
				//point_cloud.points[i].x = cpu_cloud[i][2];
				//point_cloud.points[i].y = -cpu_cloud[i][0];
				//point_cloud.points[i].z = -cpu_cloud[i][1];
				//point_cloud.points[i].rgb = cpu_cloud[i][3];
				out.points.push_back(std::vector<float>{
					cpu_cloud[i][2],
					-cpu_cloud[i][0],
					-cpu_cloud[i][1]});
				std::vector<float> channels = {};
				out.channels.push_back(channels);	
			}
			out.n_points = out.points.size();
			out.n_channels = 0;
			new_lcm.publish("DRAKE_POINTCLOUD_TEST", &out);
			printf("Published\n");
	
			
			
			
			//pcl::PCLPointCloud2 pcl_pc2
			//pcl::toPCLPointCloud2(point_cloud, pcl_pc2);

			//printf("%d, %d\n", pcl_pc2.height, pcl_pc2.width);

//			bot_core::pointcloud2_t pc2;
//			
//			// header	
//			pc2.seq = pcl_pc2.header.seq;
//			pc2.frame_id = pcl_pc2.header.frame_id;
//			// time????
//		
//			pc2.height = pcl_pc2.height;
//			pc2.width = pcl_pc2.width;
//			
//			// fields
//			//fromPCL(pcl_pc2.fields, pc2.fields);
//			pc2.fields.resize(pcl_pc2.fields.size());
//			std::vector<pcl::PCLPointField>::const_iterator it = pcl_pc2.fields.begin();
//			int i = 0;
//			for(; it != pcl_pc2.fields.end(); ++it, ++i) {
//				pc2.fields[i].name = it->name;
//				pc2.fields[i].offset = it->offset;
//				pc2.fields[i].datatype = it->datatype;
//				pc2.fields[i].count = it->count;		
//				//fromPCL(*(it), pc2.fields[i]);
//			}

//			pc2.is_bigendian = pcl_pc2.is_bigendian;
//			pc2.point_step = pcl_pc2.point_step;
//			pc2.row_step = pcl_pc2.row_step;
//			pc2.row_step = pcl_pc2.row_step;
//			pc2.data = pcl_pc2.data;
//			
//			
//			new_lcm.publish("DRAKE_POINTCLOUD_TEST", &pc2);

			

			// pcl::io::savePCDFileASCII ("test_pcd.pcd", point_cloud);
			// printf("Saved!\n");
			
		}
		else {
			sl::sleep_ms(1);	
		}
	}
}

void close() {
	quit = true;
	zed_callback.join();
	cloud.free(MEM_CPU);
	zed.close();
}
