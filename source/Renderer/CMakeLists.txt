cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_library(RendererLib
    "src/Renderer.cpp"
    "src/Inpainter.cpp"
    "src/SubBlockCuller.cpp"
    "src/AdditiveSynthesizer.cpp"
    "src/ViewingSpaceController.cpp"
    "src/reprojectPoints.cpp"
    "src/ViewWeightingSynthesizer.cpp"
    "src/MpiSynthesizer.cpp"
    "src/NoSynthesizer.cpp"
    "src/RecoverPrunedViews.cpp"
    "include/TMIV/Renderer/IRenderer.h"
    "include/TMIV/Renderer/IInpainter.h"
    "include/TMIV/Renderer/ISynthesizer.h"
    "include/TMIV/Renderer/IViewingSpaceController.h"
    "include/TMIV/Renderer/Renderer.h"
    "include/TMIV/Renderer/SubBlockCuller.h"
    "include/TMIV/Renderer/NoCuller.h"
    "include/TMIV/Renderer/ICuller.h"
    "include/TMIV/Renderer/Inpainter.h"
    "include/TMIV/Renderer/AdditiveSynthesizer.h"
    "include/TMIV/Renderer/ViewingSpaceController.h"
    "include/TMIV/Renderer/reprojectPoints.h"
    "include/TMIV/Renderer/Engine.h"
    "include/TMIV/Renderer/Engine_ERP.hpp"
    "include/TMIV/Renderer/Engine_Perspective.hpp"
    "include/TMIV/Renderer/Engine_Orthographic.hpp"
    "include/TMIV/Renderer/blend.h"
    "include/TMIV/Renderer/blend.hpp"
    "include/TMIV/Renderer/Rasterizer.h"
    "include/TMIV/Renderer/Rasterizer.hpp"
    "include/TMIV/Renderer/AccumulatingPixel.h"
    "include/TMIV/Renderer/MpiRasterizer.h"
    "include/TMIV/Renderer/MpiRasterizer.hpp"
    "include/TMIV/Renderer/ViewWeightingSynthesizer.h"
    "include/TMIV/Renderer/MpiSynthesizer.h"
    "include/TMIV/Renderer/NoSynthesizer.h"
    "include/TMIV/Renderer/RecoverPrunedViews.h"
    )

add_library(TMIV::RendererLib
    ALIAS RendererLib)

target_link_libraries(RendererLib
    PUBLIC
        TMIV::ViewingSpaceLib
    PRIVATE
        Threads::Threads
)

target_include_directories(RendererLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    
add_library(RendererFrontLib
    "src/Front/mapInputToOutputFrames.cpp"
    "src/Front/MultipleFrameRenderer.cpp"
    "include/TMIV/Renderer/Front/mapInputToOutputFrames.h"
    "include/TMIV/Renderer/Front/MultipleFrameRenderer.h"
    )

add_library(TMIV::RendererFrontLib
    ALIAS RendererFrontLib)

target_link_libraries(RendererFrontLib
    PUBLIC
        TMIV::RendererLib
        TMIV::IOLib
)

target_include_directories(RendererFrontLib
    PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    
add_executable(Renderer
    "src/Renderer.main.cpp"
    "src/registerComponents.cpp"
    )

target_link_libraries(Renderer
    PRIVATE
        TMIV::RendererFrontLib
        TMIV::DepthQualityAssessorLib)

install(
    TARGETS
        Renderer
        RendererLib
        RendererFrontLib
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

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

    add_executable(RendererFrontTest
        "src/Front/RendererFront.test.cpp"
        "src/Front/mapInputToOutputFrames.test.cpp"
        "src/Front/MultipleFrameRenderer.test.cpp"
        )

    target_link_libraries(RendererFrontTest
        PRIVATE
            Catch2::Catch2
            TMIV::RendererFrontLib)

    catch_discover_tests(RendererFrontTest)
endif()
