#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include <sl/Camera.hpp>
#include <lcm/lcm-cpp.hpp>

#include <vector>
#include <chrono>

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

	Camera zedl;
    InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_VGA;
    init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    init_params.coordinate_units = UNIT_METER;
    init_params.camera_fps = 30;

    ERROR_CODE err = zed.open(init_params);
    if(err != SUCCESS) {
		zed.close();
        exit(-1);
	}

	width = (int) zed.getResolution().width / 4;
	height = (int) zed.getResolution().height / 4;

	// Start the camera
	//startZED();

	printf("%d, %d\n", width, height);

	lcm::LCM new_lcm("udpm://239.255.76.67:7667?ttl=1");

	if(!new_lcm.good())
		return 1;	
	
	int seq_id = 0;
	while(true) {
		if(zed.grab() == SUCCESS) {
			//auto start = std::chrono::system_clock::now();
			zed.retrieveMeasure(cloud, MEASURE_XYZ, MEM_CPU, width, height);
			//auto end = std::chrono::system_clock::now();
			//std::chrono::duration<double> elapsed_secs = end - start;
			//std::cout << "Grabbed in " << elapsed_secs.count() << " secs." << std::endl;
			
			//start = std::chrono::system_clock::now();
			int size = width*height;
			
			bot_core::pointcloud_t out;
			out.seq = seq_id++;
			
			sl::Vector4<float>* cpu_cloud = cloud.getPtr<sl::float4>();
			
			for(int i = 0; i < size; i++) {
				if(cpu_cloud[i][2] > 3 || abs(cpu_cloud[i][0]) > 1.5) { // greater than 3 meter distance or left or right
					//printf("Greater\n");
					continue;
				}
//				point_cloud->points[i].x = cpu_cloud[i][2];
//				point_cloud->points[i].y = -cpu_cloud[i][0];
//				point_cloud->points[i].z = -cpu_cloud[i][1];
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
			
			//end = std::chrono::system_clock::now();
			//elapsed_secs = end - start;
			//std::cout << "Middle time: " << elapsed_secs.count() << " secs." << std::endl;
			
			//start = std::chrono::system_clock::now();
			
			auto t = std::chrono::system_clock::now();
			out.utime = std::chrono::duration_cast<std::chrono::milliseconds>(t.time_since_epoch()).count();
			new_lcm.publish("DRAKE_POINTCLOUD_TEST", &out);
			//end = std::chrono::system_clock::now();
			//elapsed_secs = end - start;
			//std::cout << "Published in " << elapsed_secs.count() << " secs." << std::endl;
			
			std::cout << "Published: " << out.seq << " @ " << out.utime << std::endl;
	
			sl::sleep_ms(25);
		}
		else {
			sl::sleep_ms(1);	
		}
	}
    return (0);
}

