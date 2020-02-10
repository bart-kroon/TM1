cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(DepthOccupancyLib
    "src/DepthOccupancy.cpp"
	"src/DepthOccupancy.reg.hpp"
	"include/TMIV/DepthOccupancy/IDepthOccupancy.h"
	"include/TMIV/DepthOccupancy/DepthOccupancy.h"
	)

add_library(TMIV::DepthOccupancyLib
	ALIAS DepthOccupancyLib)

target_link_libraries(DepthOccupancyLib
	PUBLIC
		TMIV::CommonLib
		TMIV::ImageLib
		TMIV::MivBitstreamLib)

target_include_directories(DepthOccupancyLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
	TARGETS
		DepthOccupancyLib
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if (CMAKE_TESTING_ENABLED)
	add_executable(DepthOccupancyTest "src/DepthOccupancy.test.cpp")

	target_link_libraries(DepthOccupancyTest
		PRIVATE
			Catch2::Catch2
			TMIV::DepthOccupancyLib)

	catch_discover_tests(DepthOccupancyTest)
endif()	
