cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(DepthOccupancyLib
    "src/DepthOccupancy.cpp"
	"src/DepthOccupancy.reg.hpp"
	"include/TMIV/DepthOccupancy/IDepthOccupancy.h"
	"include/TMIV/DepthOccupancy/DepthOccupancy.h")

add_library(TMIV::DepthOccupancyLib
	ALIAS DepthOccupancyLib)

target_link_libraries(DepthOccupancyLib
	PUBLIC
		TMIV::CommonLib
		TMIV::ImageLib
		TMIV::MetadataLib)

target_include_directories(DepthOccupancyLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

install(
	TARGETS
		DepthOccupancyLib
	EXPORT DepthOccupancy.Targets
	ARCHIVE DESTINATION "lib")

install(
	EXPORT DepthOccupancy.Targets
	FILE "DepthOccupancy.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")
	
if (CMAKE_TESTING_ENABLED)
	add_executable(DepthOccupancyTest "src/DepthOccupancy.test.cpp")

	target_link_libraries(DepthOccupancyTest
		PRIVATE
			Catch2::Catch2
			TMIV::DepthOccupancyLib)

	catch_discover_tests(DepthOccupancyTest)
endif()	
