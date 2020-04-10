cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(DepthQualityAssessorLib
	"src/DepthQualityAssessor.cpp"
	"src/DepthQualityAssessor.reg.hpp"
	"include/TMIV/DepthQualityAssessor/IDepthQualityAssessor.h"
	"include/TMIV/DepthQualityAssessor/DepthQualityAssessor.h"
	)

add_library(TMIV::DepthQualityAssessorLib
	ALIAS DepthQualityAssessorLib)

target_link_libraries(DepthQualityAssessorLib
	PUBLIC
		TMIV::RendererLib)

target_include_directories(DepthQualityAssessorLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
	TARGETS
		DepthQualityAssessorLib
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
