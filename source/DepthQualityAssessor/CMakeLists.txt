cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(
    TARGET
    DepthQualityAssessorLib
    SOURCES
    "src/DepthQualityAssessor.cpp"
    "src/DepthQualityAssessorStage.cpp"
    PRIVATE
    RendererLib)
