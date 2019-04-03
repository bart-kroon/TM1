cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(AtlasDeconstructorLib
	"src/IAtlasDeconstructor.cpp"
    "src/AtlasDeconstructor.cpp"
	"include/TMIV/AtlasDeconstructor/IAtlasDeconstructor.h"
	"include/TMIV/AtlasDeconstructor/AtlasDeconstructor.h")

add_library(TMIV::AtlasDeconstructorLib
	ALIAS AtlasDeconstructorLib)

target_link_libraries(AtlasDeconstructorLib
	PUBLIC
		TMIV::MetadataLib)

target_include_directories(AtlasDeconstructorLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

add_executable(AtlasDeconstructor
	"src/AtlasDeconstructor.main.cpp")

target_link_libraries(AtlasDeconstructor
	PRIVATE AtlasDeconstructorLib)

install(
	TARGETS
		AtlasDeconstructor
		AtlasDeconstructorLib
	EXPORT AtlasDeconstructor.Targets
	ARCHIVE DESTINATION "lib"
	RUNTIME DESTINATION "bin")

install(
	EXPORT AtlasDeconstructor.Targets
	FILE "AtlasDeconstructor.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")

if (CMAKE_TESTING_ENABLED)
    add_library(
		AtlasDeconstructorTest
			"src/AtlasDeconstructor.test.cpp")

    target_link_libraries(
		AtlasDeconstructorTest
		PUBLIC
			TMIV::AtlasDeconstructorLib
			TMIV::catch)
endif()

