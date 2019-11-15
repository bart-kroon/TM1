cmake_minimum_required(VERSION 3.7 FATAL_ERROR)
project(TMIV)

set(CMAKE_CXX_EXTENSIONS OFF)
include(clang_tidy.cmake)

# Read this first: https://gist.github.com/mbinna/c61dbb39bca0e4fb7d1f73b0d66a4fd1 

# To enable testing, git clone https://github.com/catchorg/Catch2
if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/Catch2)
	add_subdirectory(Catch2)
	list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/Catch2/contrib)
	enable_testing()
	include(Catch)

	# To enable spikes, add a directory Spikes
	if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/Spikes)
		add_subdirectory(Spikes)
	endif()
endif()

# Common functionnalities
add_subdirectory(Common)

# Implementation of the MIV metadata
add_subdirectory(Metadata)

# The only library that is allowed to touch the filesystem. 
# Only executables depend on this library.
add_subdirectory(IO) 

# Low-level image processing including texture and depth format conversion
add_subdirectory(Image)

add_subdirectory(ViewOptimizer)
add_subdirectory(AtlasConstructor)
add_subdirectory(AtlasDeconstructor)
add_subdirectory(DepthOccupancy)
add_subdirectory(Renderer)
add_subdirectory(ViewingSpace)

add_subdirectory(Encoder)
add_subdirectory(Decoder)

add_custom_target(clang_format COMMAND ${CMAKE_COMMAND} "-DANALYZE_DIR=${CMAKE_SOURCE_DIR}" "-P" ${CMAKE_SOURCE_DIR}/clang_format.cmake)
