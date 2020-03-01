cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(DecoderLib
    "src/Decoder.cpp"
	"src/Decoder.reg.hpp"
	"src/DepthScaler.cpp"
	"include/TMIV/Decoder/IDecoder.h"
	"include/TMIV/Decoder/Decoder.h"
	"include/TMIV/Decoder/DepthScaler.h")

add_library(TMIV::DecoderLib
	ALIAS DecoderLib)

target_link_libraries(DecoderLib
	PUBLIC
		TMIV::RendererLib)

target_include_directories(DecoderLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

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
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
