cmake_minimum_required(VERSION 3.0.2)
project(radarays_gazebo_plugins VERSION 1.0.0)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

set(GZ_RM_DEPS
  gazebo_ros
  roscpp
  sensor_msgs
  radarays_ros
  rmagine_gazebo_plugins
  cv_bridge
  image_transport
)

find_package(catkin REQUIRED COMPONENTS
  ${GZ_RM_DEPS}
)

find_package(gazebo REQUIRED)

find_package(OpenCV REQUIRED)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

find_package(rmagine 2.2.1...
COMPONENTS
  core
OPTIONAL_COMPONENTS
  embree
  cuda
  optix
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # gazebo_radarays_rotating_gzregister
  CATKIN_DEPENDS 
    ${GZ_RM_DEPS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


if(TARGET rmagine::embree)

message(WARNING "BUILD RadaRays Embree Gazebo Plugin")

### BUILD SENSOR PLUGIN ###
add_library(radarays_embree_gzplugin SHARED
  src/radarays_embree_gzplugin.cpp
)

target_link_libraries(radarays_embree_gzplugin 
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  rmagine::embree
)

### REGISTER SENSOR PLUGIN ###
add_library(radarays_embree_sensors_gzregister SHARED
  src/radarays_embree_gzregister.cpp
)

target_link_libraries(radarays_embree_sensors_gzregister
  radarays_embree_gzplugin 
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBS}
  rmagine::embree
)

# ROS Gazebo plugins
add_library(radarays_embree_ros_gzplugin SHARED 
  src/radarays_embree_ros_gzplugin.cpp)

target_link_libraries(radarays_embree_ros_gzplugin 
  radarays_embree_gzplugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  rmagine::embree
)

endif(TARGET rmagine::embree)

if(TARGET rmagine::optix)

message(WARNING "BUILD RadaRays Optix Gazebo Plugin")

### BUILD SENSOR PLUGIN ###
add_library(radarays_optix_gzplugin SHARED
  src/radarays_optix_gzplugin.cpp
)

target_link_libraries(radarays_optix_gzplugin 
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  rmagine::optix
)

### REGISTER SENSOR PLUGIN ###
add_library(radarays_optix_sensors_gzregister SHARED
  src/radarays_optix_gzregister.cpp
)

target_link_libraries(radarays_optix_sensors_gzregister
  radarays_optix_gzplugin 
  ${GAZEBO_LIBRARIES}
  ${OpenCV_LIBS}
  rmagine::optix
)

# ROS Gazebo plugins
add_library(radarays_optix_ros_gzplugin SHARED 
  src/radarays_optix_ros_gzplugin.cpp)

target_link_libraries(radarays_optix_ros_gzplugin 
  radarays_optix_gzplugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  rmagine::optix
)

endif(TARGET rmagine::optix)
