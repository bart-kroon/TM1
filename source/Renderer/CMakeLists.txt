cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

create_tmiv_library(
    TARGET
        RendererLib
    SOURCES
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
    PUBLIC
        TMIV::ViewingSpaceLib
    PRIVATE
        Threads::Threads
    )

    
create_tmiv_library(
    TARGET
        RendererFrontLib
    SOURCES
        "src/Front/mapInputToOutputFrames.cpp"
        "src/Front/MultipleFrameRenderer.cpp"
        "include/TMIV/Renderer/Front/mapInputToOutputFrames.h"
        "include/TMIV/Renderer/Front/MultipleFrameRenderer.h"
    PUBLIC
        TMIV::RendererLib
        TMIV::IOLib
    )

   
create_tmiv_executable(
    TARGET
        Renderer
    SOURCES
        "src/Renderer.main.cpp"
        "src/registerComponents.cpp"
    PRIVATE
        TMIV::RendererFrontLib
        TMIV::DepthQualityAssessorLib
    )

create_catch2_unit_test(
    TARGET
        RendererTest
    SOURCES
        "src/Renderer.test.cpp"
    PRIVATE
        TMIV::RendererLib
    )

create_catch2_unit_test(
    TARGET
        RendererFrontTest
    SOURCES
        "src/Front/RendererFront.test.cpp"
        "src/Front/mapInputToOutputFrames.test.cpp"
        "src/Front/MultipleFrameRenderer.test.cpp"
    PRIVATE
        TMIV::RendererFrontLib
    )
