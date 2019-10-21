cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

add_library(DecoderLib
    "src/Decoder.cpp"
	"src/Decoder.reg.hpp"
	"include/TMIV/Decoder/IDecoder.h"
    "include/TMIV/Decoder/Decoder.h")

add_library(TMIV::DecoderLib
	ALIAS DecoderLib)

target_link_libraries(DecoderLib
	PUBLIC
		TMIV::AtlasDeconstructorLib
		TMIV::RendererLib)

target_include_directories(DecoderLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

add_executable(Decoder
	"src/Decoder.main.cpp")

target_link_libraries(Decoder
	PRIVATE
		TMIV::DecoderLib
		TMIV::IOLib)

install(
	TARGETS
		Decoder
		DecoderLib
	EXPORT Decoder.Targets
	ARCHIVE DESTINATION "lib"
	RUNTIME DESTINATION "bin")

install(
	EXPORT Decoder.Targets
	FILE "Decoder.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")
