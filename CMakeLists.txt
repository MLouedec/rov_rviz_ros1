cmake_minimum_required(VERSION 3.0.2)
project(rov_rviz_ros1)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs usbl tf visualization_msgs dynamic_reconfigure)

# Import Eigen
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# Import Json
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

# add dynamic reconfigure api
generate_dynamic_reconfigure_options(cfg/viewer.cfg)

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
  INCLUDE_DIRS include meshs
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        usbl
        tf
        visualization_msgs
        mavros_msgs
        dynamic_reconfigure
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
   include/rov_rviz/rov_rviz_node.h src/rov_rviz_node.cpp
 )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(rov_rviz_node src/rov_rviz_node.cpp)
target_link_libraries(rov_rviz_node ${catkin_LIBRARIES} ${PROJECT_NAME} Eigen3::Eigen)
add_dependencies(rov_rviz_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(simu_reader_node src/simu_reader_node.cpp)
target_link_libraries(simu_reader_node ${catkin_LIBRARIES} ${PROJECT_NAME} Eigen3::Eigen ${JSONCPP_LIBRARIES})
add_dependencies(simu_reader_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


#############
## Install ##
#############

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
