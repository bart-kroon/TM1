cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(ViewingSpaceLib
    "src/ViewingSpaceEvaluator.cpp"
	"include/TMIV/ViewingSpace/ViewingSpaceEvaluator.h"
	"include/TMIV/ViewingSpace/SignedDistance.h")

add_library(TMIV::ViewingSpaceLib
	ALIAS ViewingSpaceLib)

target_link_libraries(ViewingSpaceLib
	PUBLIC
		TMIV::CommonLib
		TMIV::MetadataLib)

target_include_directories(ViewingSpaceLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

install(
	TARGETS
		ViewingSpaceLib
	EXPORT ViewingSpace.Targets
	ARCHIVE DESTINATION "lib")

install(
	EXPORT ViewingSpace.Targets
	FILE "ViewingSpace.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")
	
if (CMAKE_TESTING_ENABLED)
	add_executable(ViewingSpaceTest "src/ViewingSpace.test.cpp")

	target_link_libraries(ViewingSpaceTest
		PRIVATE
			Catch2::Catch2
			TMIV::ViewingSpaceLib)

	catch_discover_tests(ViewingSpaceTest)
endif()	
