cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(ImageLib
	"src/Image.cpp"
	"include/TMIV/Image/Image.h"
	"include/TMIV/Image/Image.hpp")

add_library(TMIV::ImageLib
	ALIAS ImageLib)

target_link_libraries(ImageLib
	PUBLIC
		TMIV::CommonLib
		TMIV::MetadataLib)

target_include_directories(ImageLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

install(
	TARGETS ImageLib
	EXPORT Image.Targets
	ARCHIVE DESTINATION "lib")

install(
	EXPORT Image.Targets
	FILE "Image.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")

if (CMAKE_TESTING_ENABLED)
	add_executable(ImageTest "src/Image.test.cpp")

	target_link_libraries(ImageTest
		PRIVATE
			Catch2::Catch2
			TMIV::ImageLib)

	catch_discover_tests(ImageTest)
endif()
