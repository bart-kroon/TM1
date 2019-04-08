cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(EncoderLib
	"src/IEncoder.cpp"
    "src/Encoder.cpp"
	"include/TMIV/Encoder/IEncoder.h"
    "include/TMIV/Encoder/Encoder.h")

add_library(TMIV::EncoderLib
	ALIAS EncoderLib)

target_link_libraries(EncoderLib
	PUBLIC
		TMIV::ViewOptimizerLib
		TMIV::AtlasConstructorLib)

target_include_directories(EncoderLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

add_executable(Encoder
	"src/Encoder.main.cpp")

target_link_libraries(Encoder
	PRIVATE
		TMIV::EncoderLib
		TMIV::IOLib)

install(
	TARGETS
		Encoder
		EncoderLib
	EXPORT Encoder.Targets
	ARCHIVE DESTINATION "lib"
	RUNTIME DESTINATION "bin")

install(
	EXPORT Encoder.Targets
	FILE "Encoder.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")

if (CMAKE_TESTING_ENABLED)
	add_executable(EncoderTest "src/Encoder.test.cpp")

	target_link_libraries(EncoderTest
		PRIVATE
			Catch2::Catch2
			TMIV::EncoderLib)

	catch_discover_tests(EncoderTest)
endif()
