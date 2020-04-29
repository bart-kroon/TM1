cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

if (HAVE_HM)
	set(hmSources
		"src/HmVideoDecoder.cpp"
		"include/TMIV/VideoDecoder/HmVideoDecoder.h")
endif()

add_library(VideoDecoderLib
    "src/IVideoDecoder.cpp"
	"src/VideoServer.cpp"
	"include/TMIV/VideoDecoder/IVideoDecoder.h"
	"include/TMIV/VideoDecoder/VideoServer.h"
	${hmSources}
	)

add_library(TMIV::VideoDecoderLib ALIAS VideoDecoderLib)
target_link_libraries(VideoDecoderLib PUBLIC TMIV::MivBitstreamLib)

target_include_directories(
	VideoDecoderLib
		PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

if (HAVE_HM)
	target_link_libraries(VideoDecoderLib PRIVATE TMIV::TLibDecoder)
	target_compile_definitions(VideoDecoderLib PRIVATE HAVE_HM)
endif()

install(
	TARGETS
		VideoDecoderLib
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

option(BUILD_VideoDecoder "Build TMIV exectable VideoDecoder" FALSE)

if (BUILD_VideoDecoder)
	add_executable(VideoDecoder "src/VideoDecoder.main.cpp")
	target_link_libraries(VideoDecoder PRIVATE VideoDecoderLib)

	install(
		TARGETS
			VideoDecoder
		EXPORT TMIVTargets
		RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
