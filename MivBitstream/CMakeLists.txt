cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(MivBitstreamLib
	"src/AdaptationParameterSetRBSP.cpp"
	"src/AtlasFrameParameterSetRBSP.cpp"
	"src/AtlasSequenceParameterSetRBSP.cpp"
	"src/AtlasSubBitstream.cpp"
	"src/AtlasTileGroupLayerRBSP.cpp"
	"src/IvSequenceParams.cpp"
	"src/IvAccessUnitParams.cpp"	
	"src/MivDecoder.cpp"
	"src/MivEncoder.cpp"
	"src/MivParameterSet.cpp"
	"src/MivPatchUnit.cpp"
	"src/NalSampleStreamFormat.cpp"
	"src/NalUnit.cpp"
	"src/NalSampleStream.cpp"
	"src/verify.h"
	"src/verify.cpp"
	"src/ViewingSpace.cpp"
	"src/VpccParameterSet.cpp"
	"src/VpccSampleStreamFormat.cpp"
	"src/VpccUnit.cpp"
	"include/TMIV/MivBitstream/AdaptationParameterSetRBSP.h"
	"include/TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h"
	"include/TMIV/MivBitstream/AtlasFrameParameterSetRBSP.hpp"
	"include/TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h"
	"include/TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.hpp"
	"include/TMIV/MivBitstream/AtlasSubBitstream.h"
	"include/TMIV/MivBitstream/AtlasTileGroupLayerRBSP.h"
	"include/TMIV/MivBitstream/AtlasTileGroupLayerRBSP.hpp"
	"include/TMIV/MivBitstream/DepthOccupancyTransform.h"
	"include/TMIV/MivBitstream/DepthOccupancyTransform.hpp"
	"include/TMIV/MivBitstream/EmptySyntaxStructure.h"
	"include/TMIV/MivBitstream/IvSequenceParams.h"
	"include/TMIV/MivBitstream/IvAccessUnitParams.h"
	"include/TMIV/MivBitstream/MivDecoder.h"
	"include/TMIV/MivBitstream/MivEncoder.h"
	"include/TMIV/MivBitstream/MivParameterSet.h"
	"include/TMIV/MivBitstream/MivParameterSet.hpp"
	"include/TMIV/MivBitstream/MivPatchUnit.h"
	"include/TMIV/MivBitstream/MivPatchUnit.hpp"
	"include/TMIV/MivBitstream/NalSampleStreamFormat.h"
	"include/TMIV/MivBitstream/NalUnit.h"
	"include/TMIV/MivBitstream/NalSampleStream.h"
	"include/TMIV/MivBitstream/ViewingSpace.h"
	"include/TMIV/MivBitstream/VpccParameterSet.h"
	"include/TMIV/MivBitstream/VpccParameterSet.hpp"
	"include/TMIV/MivBitstream/VpccSampleStreamFormat.h"
	"include/TMIV/MivBitstream/VpccUnit.h"
	"include/TMIV/MivBitstream/VpccUnit.hpp"
	)

add_library(TMIV::MivBitstreamLib
    ALIAS MivBitstreamLib)

target_link_libraries(MivBitstreamLib
	PUBLIC
		TMIV::CommonLib)

target_include_directories(MivBitstreamLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
	TARGETS
	    MivBitstreamLib
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if (CMAKE_TESTING_ENABLED)
	add_executable(MivBitstreamTest
		"src/AtlasFrameParameterSetRBSP.test.cpp"
		"src/AtlasSequenceParameterSetRBSP.test.cpp"
		"src/AtlasTileGroupLayerRBSP.test.cpp"
		"src/MivDecoder.test.cpp"
		"src/MivEncoder.test.cpp"
		"src/MivParameterSet.test.cpp"
		"src/MivPatchUnit.test.cpp"
		"src/NalSampleStreamFormat.test.cpp"
		"src/NalUnit.test.cpp"		
		"src/test.h"	
		"src/VpccParameterSet.test.cpp"
		"src/VpccUnit.test.cpp"
		)

	target_link_libraries(MivBitstreamTest
		PRIVATE
			Catch2::Catch2
			TMIV::MivBitstreamLib)

	catch_discover_tests(MivBitstreamTest)

	add_executable(MivBitstreamSpikes
		"src/MivBitstream.spikes.cpp")

	target_link_libraries(MivBitstreamSpikes
		PRIVATE
			Catch2::Catch2
			TMIV::MivBitstreamLib
			TMIV::IOLib)

	add_executable(VpccBitstreamSpikes
		"src/VpccBitstream.spikes.cpp")

	target_link_libraries(VpccBitstreamSpikes
		PRIVATE
			Catch2::Catch2
			TMIV::MivBitstreamLib
			TMIV::IOLib)
endif()
