cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(EncoderLib
	"src/Encoder.cpp"
	"src/Encoder_prepareSequence.cpp"
	"src/Encoder_prepareAccessUnit.cpp"
	"src/Encoder_pushFrame.cpp"
	"src/Encoder_completeAccessUnit.cpp"
	"src/Encoder_popFrame.cpp"
	"src/GroupBasedEncoder.cpp"
    "src/GeometryDownscaler.cpp"
	"src/Encoder.reg.hpp"
	"include/TMIV/Encoder/IEncoder.h"
	"include/TMIV/Encoder/Encoder.h"
	"include/TMIV/Encoder/GroupBasedEncoder.h"
	"include/TMIV/Encoder/GeometryDownscaler.h"
	)

add_library(TMIV::EncoderLib
	ALIAS EncoderLib)

target_link_libraries(EncoderLib
	PUBLIC
		TMIV::ViewOptimizerLib
		TMIV::PrunerLib
		TMIV::AggregatorLib
		TMIV::PackerLib
		TMIV::DepthOccupancyLib
		TMIV::DepthQualityAssessorLib)

target_include_directories(EncoderLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

add_executable(Encoder
	"src/Encoder.main.cpp"
	)

target_link_libraries(Encoder
	PRIVATE
		TMIV::EncoderLib
		TMIV::IOLib)

install(
	TARGETS
		Encoder
		EncoderLib
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
