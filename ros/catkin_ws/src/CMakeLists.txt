cmake_minimum_required(VERSION 2.8.3)

project(innoviz_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")

if(NOT DEFINED INVZ_DRIVER_VER OR INVZ_DRIVER_VER STREQUAL "")
message(STATUS "INVZ_DRIVER_VER not defined, using default value x.x.x")
set(INVZ_DRIVER_VER x.x.x)
else()
message(STATUS "INVZ_DRIVER_VER defines as ${INVZ_DRIVER_VER}")
endif()

# set(BINARY_DIR ${PROJECT_SOURCE_DIR}/bin)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${BINARY_DIR})
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${BINARY_DIR})
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BINARY_DIR})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  pcl_msgs
  pcl_conversions
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include 
  ${catkin_INCLUDE_DIRS}
)

add_dependencies(${TARGET}
        ${catkin_EXPORTED_TARGETS}
        )

# defines
add_definitions(-DINVZ_DRIVER_VERSION="${INVZ_DRIVER_VER}")

# set sources
set(innovizApiPath ${PROJECT_SOURCE_DIR}/../../../include/)
set(libinnovizApi_Path ${PROJECT_SOURCE_DIR}/../../../lib/linux_x86/libinnovizApi.so)

message(lib_Path: ${libinnovizApi_Path})
message (innovizApiPath: ${innovizApiPath})

add_executable(Innoviz_one_FileReader src/Innoviz_one_FileReader.cpp src/ros_utils.cpp src/invz_utils.cpp src/InvzRosIReader.cpp src/InvzRosFileReader.cpp)
target_link_libraries(Innoviz_one_FileReader ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${libinnovizApi_Path})
target_include_directories(Innoviz_one_FileReader PRIVATE ${innovizApiPath})

add_executable(Innoviz_one_Device src/Innoviz_one_Device.cpp src/ros_utils.cpp src/invz_utils.cpp src/InvzRosIReader.cpp src/InvzRosDeviceInterface.cpp)
target_link_libraries(Innoviz_one_Device ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${libinnovizApi_Path})
target_include_directories(Innoviz_one_Device PRIVATE ${innovizApiPath})

add_executable(Innoviz_one_FileReader_test src/Innoviz_one_FileReader_test.cpp src/ros_utils.cpp)
target_link_libraries(Innoviz_one_FileReader_test ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_include_directories(Innoviz_one_FileReader_test PRIVATE ${innovizApiPath})
