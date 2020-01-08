cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(MivBitstreamLib
	"src/MivDecoder.cpp"
	"src/MivEncoder.cpp"
	"src/MivParameterSet.cpp"
	"src/MivPatchUnit.cpp"
	"src/verify.h"
	"src/verify.cpp"
	"include/TMIV/MivBitstream/MivDecoder.h"
	"include/TMIV/MivBitstream/MivEncoder.h"
	"include/TMIV/MivBitstream/MivParameterSet.h"
	"include/TMIV/MivBitstream/MivParameterSet.hpp"
	"include/TMIV/MivBitstream/MivPatchUnit.h"
	"include/TMIV/MivBitstream/MivPatchUnit.hpp")

add_library(TMIV::MivBitstreamLib ALIAS MivBitstreamLib)

target_link_libraries(MivBitstreamLib
	PUBLIC
		TMIV::VpccBitstreamLib
		TMIV::MetadataLib)

target_include_directories(MivBitstreamLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

install(
	TARGETS MivBitstreamLib
	EXPORT MivBitstream.Targets
	ARCHIVE DESTINATION "lib")

install(
	EXPORT MivBitstream.Targets
	FILE "MivBitstream.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")

if (CMAKE_TESTING_ENABLED)
	add_executable(MivBitstreamTest
		"src/MivDecoder.test.cpp"
		"src/MivEncoder.test.cpp"
		"src/MivParameterSet.test.cpp"
		"src/MivPatchUnit.test.cpp"
		"src/test.h")

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

	catch_discover_tests(MivBitstreamSpikes)
endif()	