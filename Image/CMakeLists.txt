cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(ImageLib
	"src/Image.cpp"
	"include/TMIV/Image/Image.h"
	)

add_library(TMIV::ImageLib
	ALIAS ImageLib)

target_link_libraries(ImageLib
	PUBLIC
		TMIV::CommonLib)

target_include_directories(ImageLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
install(
	TARGETS
		ImageLib
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if (CMAKE_TESTING_ENABLED)
	add_executable(ImageTest "src/Image.test.cpp")

	target_link_libraries(ImageTest
		PRIVATE
			Catch2::Catch2
			TMIV::ImageLib)

	catch_discover_tests(ImageTest)
endif()
