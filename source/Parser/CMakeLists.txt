cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

create_tmiv_library(TARGET ParserLib SOURCES "src/Parser.cpp" PUBLIC MivBitstreamLib)

create_tmiv_executable(TARGET Parser SOURCES "app/Parser_main.cpp" PRIVATE ParserLib)

create_catch2_unit_test(TARGET ParserTest SOURCES "test/Parser_test.cpp" PRIVATE ParserLib)

create_tmiv_executable(TARGET BitrateReport SOURCES "app/BitrateReport_main.cpp" PRIVATE
                       MivBitstreamLib)
