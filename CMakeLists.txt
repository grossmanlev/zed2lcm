cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(zed2lcm)

SET(VERSION_REQ_CUDA "8.0")

find_package(PCL 1.2 REQUIRED)
find_package(ZED 2 REQUIRED)
find_package(lcm REQUIRED)
#include(${lcm_USE_FILE})
#find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

##For Jetson, OpenCV4Tegra is based on OpenCV2.4
SET(VERSION_REQ_CUDA "")
link_directories("/usr/lib/aarch64-linux-gnu/tegra") ## on Jetson TX1 64bits, the correct libGL.so is located here (the default one will lead to linking error (undef ref))
unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
message ("CUDA_USE_STATIC_CUDA_RUNTIME : ${CUDA_USE_STATIC_CUDA_RUNTIME}")

find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${ZED_INCLUDE_DIRS})
link_directories(${ZED_LIBRARY_DIR})

include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

# include_directories(${LCM_INCLUDE_DIRS})
# link_directories(${LCM_LIBRARY_DIR})

#add_definitions(${PCL_DEFINITIONS})

include_directories(bot_core)

add_executable(zed2lcm zed2lcm.cpp bot_core/pointcloud2_t.hpp bot_core/pointfield_t.hpp)
add_definitions(-std=c++11 -g -O3)
target_link_libraries(zed2lcm 
	${PCL_LIBRARIES}
	${ZED_LIBRARIES}
	${CUDA_CUDA_LIBRARY} 
	${CUDA_CUDART_LIBRARY} 
	${CUDA_NPP_LIBRARIES_ZED}
	lcm)

add_executable(cloud_viewer cloud_viewer.cpp)
target_link_libraries(cloud_viewer ${PCL_LIBRARIES})


#${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_npp_LIBRARY}
