cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(VpccBitstreamLib
	"src/AtlasFrameParameterSetRBSP.cpp"
	"src/AtlasSequenceParameterSetRBSP.cpp"
	"src/AtlasSubBitstream.cpp"
	"src/AtlasTileGroupLayerRBSP.cpp"
	"src/NalSampleStreamFormat.cpp"
	"src/NalUnit.cpp"
	"src/NalSampleStream.cpp"
	"src/verify.h"
	"src/verify.cpp"
	"src/VpccParameterSet.cpp"
	"src/VpccSampleStreamFormat.cpp"
	"src/VpccUnit.cpp"
	"include/TMIV/VpccBitstream/AtlasFrameParameterSetRBSP.h"
	"include/TMIV/VpccBitstream/AtlasFrameParameterSetRBSP.hpp"
	"include/TMIV/VpccBitstream/AtlasSequenceParameterSetRBSP.h"
	"include/TMIV/VpccBitstream/AtlasSequenceParameterSetRBSP.hpp"
	"include/TMIV/VpccBitstream/AtlasSubBitstream.h"
	"include/TMIV/VpccBitstream/AtlasTileGroupLayerRBSP.h"
	"include/TMIV/VpccBitstream/AtlasTileGroupLayerRBSP.hpp"
	"include/TMIV/VpccBitstream/EmptySyntaxStructure.h"
	"include/TMIV/VpccBitstream/NalSampleStreamFormat.h"
	"include/TMIV/VpccBitstream/NalUnit.h"
	"include/TMIV/VpccBitstream/NalSampleStream.h"
	"include/TMIV/VpccBitstream/VpccParameterSet.h"
	"include/TMIV/VpccBitstream/VpccParameterSet.hpp"
	"include/TMIV/VpccBitstream/VpccSampleStreamFormat.h"
	"include/TMIV/VpccBitstream/VpccUnit.h"
	"include/TMIV/VpccBitstream/VpccUnit.hpp")

add_library(TMIV::VpccBitstreamLib ALIAS VpccBitstreamLib)

target_link_libraries(VpccBitstreamLib
	PUBLIC
		TMIV::CommonLib)

target_include_directories(VpccBitstreamLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

install(
	TARGETS VpccBitstreamLib
	EXPORT VpccBitstream.Targets
	ARCHIVE DESTINATION "lib")

install(
	EXPORT VpccBitstream.Targets
	FILE "VpccBitstream.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")

if (CMAKE_TESTING_ENABLED)
	add_executable(VpccBitstreamTest
		"src/AtlasFrameParameterSetRBSP.test.cpp"
		"src/AtlasSequenceParameterSetRBSP.test.cpp"
		"src/AtlasTileGroupLayerRBSP.test.cpp"
		"src/NalSampleStreamFormat.test.cpp"
		"src/NalUnit.test.cpp"		
		"src/test.h"
		"src/VpccParameterSet.test.cpp"
		"src/VpccUnit.test.cpp")

	target_link_libraries(VpccBitstreamTest
		PRIVATE
			Catch2::Catch2
			TMIV::VpccBitstreamLib)

	catch_discover_tests(VpccBitstreamTest)

	add_executable(VpccBitstreamSpikes
		"src/VpccUnit.spikes.cpp")

	target_link_libraries(VpccBitstreamSpikes
		PRIVATE
			Catch2::Catch2
			TMIV::VpccBitstreamLib)
endif()	