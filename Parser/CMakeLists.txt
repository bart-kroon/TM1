cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

add_executable(Parser "src/Parser.main.cpp")
target_link_libraries(Parser PRIVATE TMIV::DecoderLib)

install(
	TARGETS
		Parser
	EXPORT TMIVTargets
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
