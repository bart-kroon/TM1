cmake_minimum_required(VERSION 3.22 FATAL_ERROR)

create_tmiv_library(TARGET GaInserterLib SOURCES "src/GaInserter.cpp" PUBLIC MivBitstreamLib)

create_tmiv_executable(TARGET GaInserter SOURCES "app/GaInserter_main.cpp" PRIVATE GaInserterLib)

create_catch2_unit_test(TARGET GaInserterTest SOURCES "test/GaInserter_test.cpp" PRIVATE
                        GaInserterLib)
