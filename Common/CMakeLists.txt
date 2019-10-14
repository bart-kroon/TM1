cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(CommonLib
	"src/Common.cpp"
	"src/Application.cpp"
	"src/Json.cpp"
	"src/Factory.cpp"
	"src/Frame.cpp"
	"src/LinAlg.cpp"
	"src/Vector.cpp"
	"src/Matrix.cpp"
	"src/Array.cpp"
	"src/Traits.cpp"
	"include/TMIV/Common/Common.h"
	"include/TMIV/Common/Common.hpp"
	"include/TMIV/Common/Application.h"
	"include/TMIV/Common/Json.h"
	"include/TMIV/Common/Json.hpp"
	"include/TMIV/Common/Factory.h"
    "include/TMIV/Common/Thread.h"
	"include/TMIV/Common/Frame.h"
	"include/TMIV/Common/Frame.hpp"
	"include/TMIV/Common/LinAlg.h"
	"include/TMIV/Common/LinAlg.hpp"
	"include/TMIV/Common/Vector.h"
	"include/TMIV/Common/Matrix.h"
	"include/TMIV/Common/Array.h")

add_library(TMIV::CommonLib
	ALIAS CommonLib)

target_include_directories(CommonLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

target_compile_features(CommonLib
	PUBLIC cxx_std_17)

install(
	TARGETS CommonLib
	EXPORT Common.Targets
	ARCHIVE DESTINATION "lib")

install(
	EXPORT Common.Targets
	FILE "Common.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")

if (CMAKE_TESTING_ENABLED)
	add_executable(CommonTest "src/Common.test.cpp")

	target_link_libraries(CommonTest
		PRIVATE
			Catch2::Catch2
			TMIV::CommonLib)

	catch_discover_tests(CommonTest WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..)
endif()
