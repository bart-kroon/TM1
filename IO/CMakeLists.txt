cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(IOLib
	"src/IO.cpp"
	"src/IvMetadataReader.cpp"
	"src/IvMetadataWriter.cpp"
	"include/TMIV/IO/IO.h"
	"include/TMIV/IO/IvMetadataReader.h"
	"include/TMIV/IO/IvMetadataWriter.h"
	)

add_library(TMIV::IOLib
	ALIAS IOLib)

target_link_libraries(IOLib
	PUBLIC
		TMIV::CommonLib
		TMIV::MetadataLib
		TMIV::ViewOptimizerLib
		TMIV::ImageLib)

target_include_directories(IOLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
	TARGETS
		IOLib
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
