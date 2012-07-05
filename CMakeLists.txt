cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

find_package(PCL 1.5 REQUIRED)

set(CMAKE_BUILD_TYPE DEBUG)

find_package(Boost)
if (boost_found)
	message ("Boost library found")
	message (${BOOST_INCLUDE_DIR})
	include_directories(${BOOST_INCLUDE_DIR})
	add_definitions("-dhas_boost")
endif()

include_directories(${PCL_INCLUDE_DIRS} /usr/local/include)
link_directories(${PCL_LIBRARY_DIRS} /usr/local/lib/mysql /usr/lib)
add_definitions(${PCL_DEFINITIONS})

MESSAGE("PCL includes - ${PCL_INCLUDE_DIRS}")
MESSAGE("PCL Definitions - ${PCL_DEFINITIONS}")

add_executable (texttopcd texttopcd.cpp)
add_executable (iterative_closest_point iterative_closest_point.cpp)
add_executable (registration registration.cpp)
add_executable (get_displacements get_displacements.cpp)

target_link_libraries (texttopcd boost_program_options ${PCL_LIBRARIES})
target_link_libraries (iterative_closest_point boost_program_options ${PCL_LIBRARIES})
target_link_libraries (registration flann boost_program_options ${PCL_LIBRARIES})
target_link_libraries (get_displacements boost_program_options ${PCL_LIBRARIES})
