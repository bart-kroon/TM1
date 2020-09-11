cmake_minimum_required(VERSION 3.14 FATAL_ERROR)

add_executable(Parser "src/Parser.main.cpp")
target_link_libraries(Parser PRIVATE TMIV::MivBitstreamLib)

add_executable(BitrateReport "src/BitrateReport.main.cpp")
target_link_libraries(BitrateReport PRIVATE TMIV::MivBitstreamLib)

install(
    TARGETS
        Parser
        BitrateReport
    EXPORT TMIVTargets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
