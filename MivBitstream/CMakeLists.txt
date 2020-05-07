cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(MivBitstreamLib
	"src/AccessUnit.cpp"
	"src/AccessUnitDelimiterRBSP.cpp"
	"src/AtlasAdaptationParameterSetRBSP.cpp"
	"src/AtlasFrameParameterSetRBSP.cpp"
	"src/AtlasSequenceParameterSetRBSP.cpp"
	"src/AtlasTileLayerRBSP.cpp"
	"src/BitrateReport.h"
	"src/FrameOrderCountRBSP.cpp"
	"src/IvSequenceParams.cpp"
	"src/IvAccessUnitParams.cpp"	
	"src/MivDecoder.cpp"
	"src/MivEncoder.cpp"
	"src/MivVuiParams.cpp"
	"src/NalSampleStreamFormat.cpp"
	"src/NalUnit.cpp"
	"src/NalSampleStream.cpp"
	"src/PatchParamsList.cpp"
	"src/RecViewport.cpp"
	"src/SeiRBSP.cpp"
	"src/verify.h"
	"src/verify.cpp"
	"src/ViewingSpace.cpp"
	"src/ViewingSpaceHandling.cpp"
	"src/ViewParamsList.cpp"
	"src/V3cParameterSet.cpp"
	"src/V3cSampleStreamFormat.cpp"
	"src/V3cUnit.cpp"
	"include/TMIV/MivBitstream/AccessUnit.h"
	"include/TMIV/MivBitstream/AccessUnitDelimiterRBSP.h"
	"include/TMIV/MivBitstream/AccessUnitDelimiterRBSP.hpp"
	"include/TMIV/MivBitstream/AtlasAdaptationParameterSetRBSP.h"
	"include/TMIV/MivBitstream/AtlasAdaptationParameterSetRBSP.hpp"
	"include/TMIV/MivBitstream/AtlasFrameParameterSetRBSP.h"
	"include/TMIV/MivBitstream/AtlasFrameParameterSetRBSP.hpp"
	"include/TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.h"
	"include/TMIV/MivBitstream/AtlasSequenceParameterSetRBSP.hpp"
	"include/TMIV/MivBitstream/AtlasSubBitstream.h"
	"include/TMIV/MivBitstream/AtlasTileLayerRBSP.h"
	"include/TMIV/MivBitstream/AtlasTileLayerRBSP.hpp"
	"include/TMIV/MivBitstream/DepthOccupancyTransform.h"
	"include/TMIV/MivBitstream/DepthOccupancyTransform.hpp"
	"include/TMIV/MivBitstream/FrameOrderCountRBSP.h"
	"include/TMIV/MivBitstream/FrameOrderCountRBSP.hpp"
	"include/TMIV/MivBitstream/IvSequenceParams.h"
	"include/TMIV/MivBitstream/IvAccessUnitParams.h"
	"include/TMIV/MivBitstream/IvAccessUnitParams.hpp"
	"include/TMIV/MivBitstream/MivDecoder.h"
	"include/TMIV/MivBitstream/MivEncoder.h"
	"include/TMIV/MivBitstream/MivVuiParams.h"
	"include/TMIV/MivBitstream/MivVuiParams.hpp"
	"include/TMIV/MivBitstream/NalSampleStreamFormat.h"
	"include/TMIV/MivBitstream/NalUnit.h"
	"include/TMIV/MivBitstream/NalSampleStream.h"
	"include/TMIV/MivBitstream/PatchParamsList.h"
	"include/TMIV/MivBitstream/RecViewport.h"
	"include/TMIV/MivBitstream/RecViewport.hpp"
	"include/TMIV/MivBitstream/SeiRBSP.h"
	"include/TMIV/MivBitstream/ViewingSpace.h"
	"include/TMIV/MivBitstream/ViewingSpaceHandling.h"
	"include/TMIV/MivBitstream/ViewParamsList.h"
	"include/TMIV/MivBitstream/V3cParameterSet.h"
	"include/TMIV/MivBitstream/V3cParameterSet.hpp"
	"include/TMIV/MivBitstream/V3cSampleStreamFormat.h"
	"include/TMIV/MivBitstream/V3cUnit.h"
	"include/TMIV/MivBitstream/V3cUnit.hpp"
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
		"src/AccessUnitDelimiterRBSP.test.cpp"
		"src/AtlasAdaptationParameterSetRBSP.test.cpp"
		"src/AtlasFrameParameterSetRBSP.test.cpp"
		"src/AtlasSequenceParameterSetRBSP.test.cpp"
		"src/AtlasTileLayerRBSP.test.cpp"
		"src/FrameOrderCountRBSP.test.cpp"
		"src/MivDecoder.test.cpp"
		"src/MivEncoder.test.cpp"
		"src/MivVuiParams.test.cpp"
		"src/NalSampleStreamFormat.test.cpp"
		"src/NalUnit.test.cpp"
		"src/RecViewport.test.cpp"		
		"src/SeiRBSP.test.cpp"
		"src/test.cpp"
		"src/test.h"
		"src/ViewingSpace.test.cpp"
		"src/ViewingSpaceHandling.test.cpp"
		"src/V3cParameterSet.test.cpp"
		"src/V3cUnit.test.cpp"
		)

	target_link_libraries(MivBitstreamTest
		PRIVATE
			Catch2::Catch2
			TMIV::MivBitstreamLib)

	catch_discover_tests(MivBitstreamTest)

	add_executable(V3cBitstreamTest
		"src/V3cBitstream.test.cpp")

	target_link_libraries(V3cBitstreamTest
		PRIVATE
			Catch2::Catch2
			TMIV::IOLib)

	catch_discover_tests(V3cBitstreamTest)
endif()
