cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(VideoDecoderLib
    "src/IVideoDecoder.cpp"
	"src/VideoServer.cpp"
	"src/HmVideoDecoder.cpp"
	"include/TMIV/VideoDecoder/IVideoDecoder.h"
	"include/TMIV/VideoDecoder/VideoServer.h"
	"include/TMIV/VideoDecoder/HmVideoDecoder.h"
	)

add_library(TMIV::VideoDecoderLib
	ALIAS VideoDecoderLib)

target_link_libraries(VideoDecoderLib
	PUBLIC
		TMIV::MivBitstreamLib
	PRIVATE
		TMIV::TLibDecoder)

target_include_directories(VideoDecoderLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

install(
	TARGETS
		VideoDecoderLib
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
