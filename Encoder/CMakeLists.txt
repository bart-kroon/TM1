cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_library(EncoderLib
	"src/AtlasConstructor.cpp"
	"src/GroupBasedEncoder.cpp"
    "src/GeometryDownscaler.cpp"
	"src/Encoder.reg.hpp"
	"include/TMIV/Encoder/IEncoder.h"
	"include/TMIV/Encoder/GroupBasedEncoder.h"
	"include/TMIV/Encoder/GeometryDownscaler.h"
	"include/TMIV/Encoder/AtlasConstructor.h"
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
	"../ctc_config/make_tmiv_configs.py"
	"../ctc_config/qp_tuning_configs.py")

target_link_libraries(Encoder
	PRIVATE
		TMIV::EncoderLib
		TMIV::DecoderLib
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
