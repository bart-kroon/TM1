cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(MetadataLib
	"src/AtlasParametersList.cpp"
	"src/CameraParametersList.cpp"
	"include/TMIV/Metadata/AtlasParametersList.h"
	"include/TMIV/Metadata/CameraParametersList.h")

add_library(TMIV::MetadataLib
	ALIAS MetadataLib)

target_link_libraries(MetadataLib
	PUBLIC TMIV::CommonLib)

target_include_directories(MetadataLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

install(
	TARGETS MetadataLib
	EXPORT Metadata.Targets
	ARCHIVE DESTINATION "lib")

install(
	EXPORT Metadata.Targets
	FILE "Metadata.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")

if (CMAKE_TESTING_ENABLED)
	add_executable(MetadataTest "src/Metadata.test.cpp")

	target_link_libraries(MetadataTest
		PRIVATE
			Catch2::Catch2
			TMIV::MetadataLib)

	catch_discover_tests(MetadataTest)
endif()	