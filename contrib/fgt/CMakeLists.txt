cmake_minimum_required(VERSION 2.8.3)
project(fgt_ros)

find_package(catkin REQUIRED)
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION})

catkin_package(
    INCLUDE_DIRS
        source/include
        ${EIGEN3_INCLUDE_DIR}
        ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}
    LIBRARIES fgt
)

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

add_subdirectory(source)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/source/include)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/source/include)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/source/CMake )

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


