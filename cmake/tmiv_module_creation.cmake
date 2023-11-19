function(create_tmiv_library)
    set(prefix TMIV_LIB_CREATOR)
    set(flags)
    set(singleValues TARGET)
    set(multiValues SOURCES PUBLIC PRIVATE)

    include(CMakeParseArguments)
    cmake_parse_arguments(${prefix} "${flags}" "${singleValues}" "${multiValues}" ${ARGN})

    if(NOT "${TMIV_BUILD_LIB_LIST}" STREQUAL "")
        set(found_library OFF)

        foreach(enabled_lib ${TMIV_BUILD_LIB_LIST})
            if(${enabled_lib} STREQUAL ${TMIV_LIB_CREATOR_TARGET})
                set(found_library ON)
                break()
            endif()
        endforeach()

        if(NOT found_library)
            message(STATUS "TMIV: Library ${TMIV_LIB_CREATOR_TARGET} is excluded from the build.")
            return()
        endif()
    endif()

    if(NOT TARGET ${TMIV_LIB_CREATOR_TARGET})
        add_library(${TMIV_LIB_CREATOR_TARGET} ${TMIV_LIB_CREATOR_SOURCES})
        set_property(TARGET ${TMIV_LIB_CREATOR_TARGET} PROPERTY FOLDER "TMIV libraries")
        target_link_libraries(${TMIV_LIB_CREATOR_TARGET} PUBLIC ${TMIV_LIB_CREATOR_PUBLIC})
        target_link_libraries(${TMIV_LIB_CREATOR_TARGET} PRIVATE ${TMIV_FMT_TARGET}
                                                                 ${TMIV_LIB_CREATOR_PRIVATE})

        target_include_directories(${TMIV_LIB_CREATOR_TARGET}
                                   PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
    endif()

    install(
        TARGETS ${TMIV_LIB_CREATOR_TARGET}
        EXPORT TMIVTargets
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})

    install(DIRECTORY "include/" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
endfunction()

function(create_tmiv_executable)
    set(prefix TMIV_EXE_CREATOR)
    set(flags)
    set(singleValues TARGET)
    set(multiValues SOURCES PRIVATE)

    include(CMakeParseArguments)
    cmake_parse_arguments(${prefix} "${flags}" "${singleValues}" "${multiValues}" ${ARGN})

    if(NOT "${TMIV_BUILD_APP_LIST}" STREQUAL "")
        set(found_app OFF)

        foreach(enabled_app ${TMIV_BUILD_APP_LIST})
            if(${enabled_app} STREQUAL ${TMIV_EXE_CREATOR_TARGET})
                set(found_app ON)
                break()
            endif()
        endforeach()

        if(NOT found_app)
            message(
                STATUS "TMIV: Executable ${TMIV_EXE_CREATOR_TARGET} is excluded from the build.")
            return()
        endif()
    endif()

    if(NOT TARGET Tmiv${TMIV_EXE_CREATOR_TARGET})
        add_executable(Tmiv${TMIV_EXE_CREATOR_TARGET} ${TMIV_EXE_CREATOR_SOURCES})
        set_property(TARGET Tmiv${TMIV_EXE_CREATOR_TARGET} PROPERTY FOLDER "TMIV executables")
        target_link_libraries(Tmiv${TMIV_EXE_CREATOR_TARGET} PRIVATE ${TMIV_FMT_TARGET}
                                                                     ${TMIV_EXE_CREATOR_PRIVATE})
    endif()

    install(
        TARGETS Tmiv${TMIV_EXE_CREATOR_TARGET}
        EXPORT TMIVTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endfunction()
