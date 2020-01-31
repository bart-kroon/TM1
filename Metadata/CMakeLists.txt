cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(MetadataLib
	"src/Bitstream.cpp"
	"src/IvSequenceParams.cpp"
	"src/IvAccessUnitParams.cpp"
	"src/ViewingSpace.cpp"
	"src/verify.cpp"
	"include/TMIV/Metadata/Bitstream.h"
	"include/TMIV/Metadata/IvSequenceParams.h"
	"include/TMIV/Metadata/IvAccessUnitParams.h"
	"include/TMIV/Metadata/DepthOccupancyTransform.h"
	"include/TMIV/Metadata/DepthOccupancyTransform.hpp"
	"include/TMIV/Metadata/ViewingSpace.h"
	)

add_library(TMIV::MetadataLib
	ALIAS MetadataLib)

target_link_libraries(MetadataLib
	PUBLIC TMIV::CommonLib)

target_include_directories(MetadataLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
install(
	TARGETS
		MetadataLib
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if (CMAKE_TESTING_ENABLED)
	add_executable(MetadataTest "src/Metadata.test.cpp")

	target_link_libraries(MetadataTest
		PRIVATE
			Catch2::Catch2
			TMIV::MetadataLib)

	catch_discover_tests(MetadataTest)
endif()	