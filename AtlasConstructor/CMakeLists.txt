cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(AtlasConstructorLib
	"src/IAtlasConstructor.cpp"
    "src/AtlasConstructor.cpp"
	"src/IPruner.cpp"
	"src/Pruner.cpp"
	"src/IAggregator.cpp"
	"src/Aggregator.cpp"
	"src/IPacker.cpp"
	"src/Cluster.cpp"
	"src/MaxRectPiP.cpp"
	"src/Packer.cpp"
	"include/TMIV/AtlasConstructor/IAtlasConstructor.h"
	"include/TMIV/AtlasConstructor/AtlasConstructor.h"
	"include/TMIV/AtlasConstructor/IPruner.h"
	"include/TMIV/AtlasConstructor/Pruner.h"
	"include/TMIV/AtlasConstructor/IAggregator.h"
	"include/TMIV/AtlasConstructor/Aggregator.h"
	"include/TMIV/AtlasConstructor/IPacker.h"
	"include/TMIV/AtlasConstructor/Packer.h")

add_library(TMIV::AtlasConstructorLib
	ALIAS AtlasConstructorLib)

target_link_libraries(AtlasConstructorLib
	PUBLIC
		TMIV::MetadataLib)

target_include_directories(AtlasConstructorLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

add_executable(AtlasConstructor
	"src/AtlasConstructor.main.cpp")

target_link_libraries(AtlasConstructor
	PRIVATE AtlasConstructorLib)

install(
	TARGETS
		AtlasConstructor
		AtlasConstructorLib
	EXPORT AtlasConstructor.Targets
	ARCHIVE DESTINATION "lib"
	RUNTIME DESTINATION "bin")

install(
	EXPORT AtlasConstructor.Targets
	FILE "AtlasConstructor.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")
