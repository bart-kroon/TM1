cmake_minimum_required(VERSION 3.7 FATAL_ERROR)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(RendererLib
	"src/IRenderer.cpp"
	"src/IInpainter.cpp"
	"src/ISynthesizer.cpp"
    "src/MultipassRenderer.cpp"
	"src/Inpainter.cpp"
	"src/Synthesizer.cpp"
	"src/reprojectPoints.cpp"
	"src/AccumulatingPixel.cpp"
	"src/AccumulatingView.cpp"
	"include/TMIV/Renderer/IRenderer.h"
	"include/TMIV/Renderer/IInpainter.h"
	"include/TMIV/Renderer/ISynthesizer.h"
    "include/TMIV/Renderer/MultipassRenderer.h"
	"include/TMIV/Renderer/Inpainter.h"
	"include/TMIV/Renderer/Synthesizer.h"
	"include/TMIV/Renderer/reprojectPoints.h")

add_library(TMIV::RendererLib
	ALIAS RendererLib)

target_link_libraries(RendererLib
	PUBLIC
		TMIV::CommonLib
		TMIV::MetadataLib
		TMIV::ImageLib		
	PRIVATE Threads::Threads)

target_include_directories(RendererLib
	PUBLIC
		"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
		"$<INSTALL_INTERFACE:include>")

add_executable(Renderer
	"src/Renderer.main.cpp")

target_link_libraries(Renderer
	PRIVATE
		TMIV::RendererLib
		TMIV::IOLib)

install(
	TARGETS
		Renderer
		RendererLib
	EXPORT Renderer.Targets
	ARCHIVE DESTINATION "lib"
	RUNTIME DESTINATION "bin")

install(
	EXPORT Renderer.Targets
	FILE "Renderer.cmake"
	NAMESPACE TMIV::
	DESTINATION "lib/cmake/TMIV")

install(
	DIRECTORY "include/"
	DESTINATION "include")
	
if (CMAKE_TESTING_ENABLED)
	add_executable(RendererTest "src/Renderer.test.cpp")

	target_link_libraries(RendererTest
		PRIVATE
			Catch2::Catch2
			TMIV::RendererLib)

	catch_discover_tests(RendererTest)
endif()
