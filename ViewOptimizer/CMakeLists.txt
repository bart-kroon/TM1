cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(ViewOptimizerLib
	"src/IViewOptimizer.cpp"
    "src/ViewReducer.cpp"
	"include/TMIV/ViewOptimizer/IViewOptimizer.h"
    "include/TMIV/ViewOptimizer/ViewReducer.h")

add_library(TMIV::ViewOptimizerLib
	ALIAS ViewOptimizerLib)

target_link_libraries(ViewOptimizerLib
	PUBLIC TMIV::CommonLib
	PUBLIC TMIV::MetadataLib)

target_include_directories(ViewOptimizerLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

add_executable(ViewOptimizer
	"src/ViewOptimizer.main.cpp")

target_link_libraries(ViewOptimizer
	PRIVATE ViewOptimizerLib)

install(
	TARGETS
		ViewOptimizer
		ViewOptimizerLib
	EXPORT ViewOptimizer.Targets
	ARCHIVE DESTINATION "lib"
	RUNTIME DESTINATION "bin")

install(
	EXPORT ViewOptimizer.Targets
	FILE "ViewOptimizer.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")
