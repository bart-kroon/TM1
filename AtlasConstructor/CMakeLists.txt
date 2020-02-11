cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(AtlasConstructorLib
    "src/AtlasConstructor.cpp"
    "src/EntityBasedAtlasConstructor.cpp"
	"src/HierarchicalPruner.cpp"
	"src/PrunedMesh.cpp"
	"src/PrunedMesh.h"
	"src/Aggregator.cpp"
	"src/Cluster.cpp"
	"src/Cluster.h"
	"src/MaxRectPiP.cpp"
	"src/MaxRectPiP.h"
	"src/Packer.cpp"
	"src/AtlasConstructor.reg.hpp"
	"include/TMIV/AtlasConstructor/IAtlasConstructor.h"
	"include/TMIV/AtlasConstructor/AtlasConstructor.h"
	"include/TMIV/AtlasConstructor/EntityBasedAtlasConstructor.h"
	"include/TMIV/AtlasConstructor/IPruner.h"
	"include/TMIV/AtlasConstructor/HierarchicalPruner.h"
	"include/TMIV/AtlasConstructor/IAggregator.h"
	"include/TMIV/AtlasConstructor/Aggregator.h"
	"include/TMIV/AtlasConstructor/IPacker.h"
	"include/TMIV/AtlasConstructor/Packer.h"
	)

add_library(TMIV::AtlasConstructorLib
	ALIAS AtlasConstructorLib)

target_link_libraries(AtlasConstructorLib
	PUBLIC
		TMIV::MetadataLib
		TMIV::RendererLib)

target_include_directories(AtlasConstructorLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
	TARGETS
		AtlasConstructorLib
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
