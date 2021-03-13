cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

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
        "src/PushPullInpainter.cpp"
        "src/RecoverPrunedViews.cpp"
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
        "src/AccumulatingPixel.test.cpp"
        "src/AffineTransform.test.cpp"
        "src/Engine_equirectangular.test.cpp"
        "src/Engine_perspective.test.cpp"
        "src/Engine_orthographic.test.cpp"
        "src/MpiRasterizer.test.cpp"
        "src/Rasterizer.test.cpp"
        "src/PushPull.test.cpp"
        "src/PushPullInpainter.test.cpp"
    PRIVATE
        TMIV::RendererLib
    )

create_catch2_unit_test(
    TARGET
        RendererFrontTest
    SOURCES
        "src/Front/mapInputToOutputFrames.test.cpp"
        "src/Front/MultipleFrameRenderer.test.cpp"
    PRIVATE
        TMIV::RendererFrontLib
    )
