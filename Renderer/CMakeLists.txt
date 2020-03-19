cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(RendererLib
    "src/Renderer.cpp"
    "src/MultipassRenderer.cpp"
    "src/GroupBasedRenderer.cpp"
	"src/Inpainter.cpp"
	"src/SubBlockCuller.cpp"
	"src/AdditiveSynthesizer.cpp"
	"src/ViewingSpaceController.cpp"
	"src/reprojectPoints.cpp"
	"src/ViewWeightingSynthesizer.cpp"
	"src/NoSynthesizer.cpp"
	"src/RecoverPrunedViews.cpp"
	"src/Renderer.reg.hpp"
	"include/TMIV/Renderer/IRenderer.h"
	"include/TMIV/Renderer/IInpainter.h"
	"include/TMIV/Renderer/ISynthesizer.h"
	"include/TMIV/Renderer/IViewingSpaceController.h"
    "include/TMIV/Renderer/Renderer.h"
    "include/TMIV/Renderer/MultipassRenderer.h"
    "include/TMIV/Renderer/GroupBasedRenderer.h"
	"include/TMIV/Renderer/SubBlockCuller.h"
	"include/TMIV/Renderer/NoCuller.h"
	"include/TMIV/Renderer/ICuller.h"
	"include/TMIV/Renderer/Inpainter.h"
	"include/TMIV/Renderer/AdditiveSynthesizer.h"
	"include/TMIV/Renderer/ViewingSpaceController.h"
	"include/TMIV/Renderer/reprojectPoints.h"
	"include/TMIV/Renderer/reprojectPoints.hpp"
	"include/TMIV/Renderer/Engine.h"
	"include/TMIV/Renderer/Engine_ERP.hpp"
	"include/TMIV/Renderer/Engine_Perspective.hpp"
	"include/TMIV/Renderer/blend.h"
	"include/TMIV/Renderer/blend.hpp"
	"include/TMIV/Renderer/Rasterizer.h"
	"include/TMIV/Renderer/Rasterizer.hpp"
	"include/TMIV/Renderer/AccumulatingPixel.h"
	"include/TMIV/Renderer/ViewWeightingSynthesizer.h"
	"include/TMIV/Renderer/NoSynthesizer.h"
	"include/TMIV/Renderer/RecoverPrunedViews.h"
	)

add_library(TMIV::RendererLib
	ALIAS RendererLib)

target_link_libraries(RendererLib
	PUBLIC
		TMIV::CommonLib
		TMIV::MivBitstreamLib
		TMIV::ViewingSpaceLib
	PRIVATE
		Threads::Threads
)

target_include_directories(RendererLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
	
install(
	TARGETS
		RendererLib
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

install(
	DIRECTORY "include/"
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

if (CMAKE_TESTING_ENABLED)
	add_executable(RendererTest "src/Renderer.test.cpp")

	target_link_libraries(RendererTest
		PRIVATE
			Catch2::Catch2
			TMIV::RendererLib)

	catch_discover_tests(RendererTest)
endif()
