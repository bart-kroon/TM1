cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(AtlasDeconstructorLib
    "src/AtlasDeconstructor.cpp"
	"src/AtlasDeconstructor.reg.hpp"
	"include/TMIV/AtlasDeconstructor/IAtlasDeconstructor.h"
	"include/TMIV/AtlasDeconstructor/AtlasDeconstructor.h"
	)

add_library(TMIV::AtlasDeconstructorLib
	ALIAS AtlasDeconstructorLib)

target_link_libraries(AtlasDeconstructorLib
	PUBLIC
		TMIV::MivBitstreamLib
		TMIV::ImageLib)

target_include_directories(AtlasDeconstructorLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
	TARGETS
		AtlasDeconstructorLib
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
