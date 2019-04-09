cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(IOLib
	"src/IO.cpp"
	"include/TMIV/IO/IO.h")

add_library(TMIV::IOLib
	ALIAS IOLib)

target_link_libraries(IOLib
	PUBLIC
		TMIV::CommonLib
		TMIV::MetadataLib
		TMIV::ViewOptimizerLib)

target_include_directories(IOLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

install(
	TARGETS IOLib
	EXPORT IO.Targets
	ARCHIVE DESTINATION "lib")

install(
	EXPORT IO.Targets
	FILE "IO.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")
