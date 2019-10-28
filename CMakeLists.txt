cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(TFAC-master)

set(CMAKE_BUILD_TYPE Release) 
find_package(PCL 1.7 REQUIRED)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS} include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

FILE(GLOB SRCS src/*.cpp)


add_executable (TFAC-master ${SRCS})
target_link_libraries (TFAC-master ${PCL_LIBRARIES})
