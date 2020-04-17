cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(
	TMIV
	DESCRIPTION "Test Model for Immersive Video"
	VERSION 4.0.0
	LANGUAGES C CXX)

set(CMAKE_CXX_EXTENSIONS OFF)

include(CMake/clang_tidy.cmake)

# To enable testing, clone https://github.com/catchorg/Catch2 2.10.2 or newer, build and install.
# Then in the CMake GUI set Catch2_DIR to the lib/cmake/Catch2 directory of the Catch2 installation.
find_package(Catch2 2.10.2 QUIET)
if (Catch2_FOUND)
	enable_testing()
	include(Catch)
endif()

include(GNUInstallDirs)
include(CMake/hm.cmake)

foreach(subdir
		Common 
		MivBitstream 
		IO
		ViewOptimizer
		AtlasConstructor
		DepthOccupancy
		Renderer
		ViewingSpace		
		VideoDecoder
		Encoder
		Decoder
		Parser
		DepthQualityAssessor)
	add_subdirectory(${subdir})
endforeach()

add_custom_target(clang_format
	COMMAND
		${CMAKE_COMMAND}
		"-DANALYZE_DIR=${CMAKE_SOURCE_DIR}"
		"-P"
		"${CMAKE_CURRENT_LIST_DIR}/CMake/clang_format.cmake")

# To use TMIV as a sub project, clone, build and install.
# Then in the CMake GUI set TMIV_DIR to the lib/cmake/TMIV directory of the TMIV installation.
# In the super project CMakeLists.txt use find_package(TMIV 4.0.0 REQUIRED)
set(TMIV_CMAKE_CONFIG_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/TMIV")

install(
	EXPORT TMIVTargets
	FILE "TMIVTargets.cmake"
	NAMESPACE TMIV::
	DESTINATION ${TMIV_CMAKE_CONFIG_DESTINATION})
export(EXPORT TMIVTargets NAMESPACE TMIV::)

include(CMakePackageConfigHelpers)

configure_package_config_file(
	"${CMAKE_CURRENT_LIST_DIR}/CMake/TMIVConfig.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/TMIVConfig.cmake"
	INSTALL_DESTINATION ${TMIV_CMAKE_CONFIG_DESTINATION})

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/TMIVConfigVersion.cmake"
	VERSION ${TMIV_VERSION}
	COMPATIBILITY SameMajorVersion)

install(
	FILES
		"${CMAKE_CURRENT_BINARY_DIR}/TMIVConfig.cmake"
		"${CMAKE_CURRENT_BINARY_DIR}/TMIVConfigVersion.cmake"
	DESTINATION ${TMIV_CMAKE_CONFIG_DESTINATION})
