cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(CommonLib
	"src/Application.cpp"
	"src/Bitstream.cpp"
	"src/Bytestream.cpp"
	"src/Json.cpp"
	"src/Half.cpp"
	"src/Frame.cpp"
	"src/Transformation.cpp"
	"src/Graph.cpp"
	"include/TMIV/Common/Bitstream.h"
	"include/TMIV/Common/Bytestream.h"
	"include/TMIV/Common/Common.h"
	"include/TMIV/Common/Common.hpp"
	"include/TMIV/Common/Application.h"
	"include/TMIV/Common/Json.h"
	"include/TMIV/Common/Json.hpp"
	"include/TMIV/Common/Factory.h"
    "include/TMIV/Common/Thread.h"
	"include/TMIV/Common/Frame.h"
	"include/TMIV/Common/Frame.hpp"
	"include/TMIV/Common/Half.h"
	"include/TMIV/Common/Half.hpp"
	"include/TMIV/Common/LinAlg.h"
	"include/TMIV/Common/LinAlg.hpp"
	"include/TMIV/Common/Transformation.h"
	"include/TMIV/Common/Vector.h"
	"include/TMIV/Common/Matrix.h"
	"include/TMIV/Common/Array.h"
	"include/TMIV/Common/Graph.h"
	"include/TMIV/Common/Quaternion.h"
	)

add_library(TMIV::CommonLib
	ALIAS CommonLib)

target_include_directories(CommonLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

target_compile_features(CommonLib
	PUBLIC cxx_std_17)

install(
	TARGETS
		CommonLib
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if (CMAKE_TESTING_ENABLED)
	add_executable(CommonTest
		"src/Bitstream.test.cpp"
		"src/Quaternion.test.cpp"
		"src/Common.test.cpp")

	target_link_libraries(CommonTest
		PRIVATE
			Catch2::Catch2
			TMIV::CommonLib)

	catch_discover_tests(CommonTest)
endif()
