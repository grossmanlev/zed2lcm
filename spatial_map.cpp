#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <sl/Camera.hpp>
#include <lcm/lcm-cpp.hpp>

#include <vector>
#include <chrono>

using namespace sl;

int main (int argc, char const* argv[])
{
	
	Camera zed;
	
	// Set configuration parameters
	sl::InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_VGA; // Use HD720 video mode (default fps: 60)
	//init_params.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system
	init_params.coordinate_units = UNIT_METER; // Set units in meters
	init_params.camera_fps = 60;
	
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS)
        exit(-1);
        
    sl::TrackingParameters tracking_parameters;
    err = zed.enableTracking(tracking_parameters);
    if (err != SUCCESS)
        exit(-1);

	// Configure spatial mapping parameters
	sl::SpatialMappingParameters mapping_parameters;
	//mapping_parameters.resolution = MAPPING_RESOLUTION_LOW;
	//mapping_parameters.range = MAPPING_RANGE_MEDIUM;

	mapping_parameters.save_texture = false;
    err = zed.enableSpatialMapping(mapping_parameters);
    if (err != SUCCESS)
        exit(-1);

	// Request an updated mesh every 0.5s
	sl::Mesh mesh; // Create a mesh object
	int timer=0;
	while (1) {
	  if (zed.grab() == SUCCESS) {

		  // Request an update of the mesh every 30 frames (0.5s in HD720 mode)
		  if (timer%30 == 0)
		     zed.requestMeshAsync();

		  // Retrieve mesh when ready
		  if (zed.getMeshRequestStatusAsync() == SUCCESS && timer > 0) {
		     zed.retrieveMeshAsync(mesh);
		     printf("Got mesh!\n");
		  }

		  timer++;
		  printf("...\n");
	  }      
	}
	return 0;
}
