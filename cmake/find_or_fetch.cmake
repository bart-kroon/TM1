function(find_or_fetch)
    cmake_parse_arguments("ARG" "REQUIRED" "NAME;VERSION;GIT_URL;GIT_REF;BUILD_FLAG" "" ${ARGN})

    set(FIND_OR_FETCH_FOUND OFF)
    set(FIND_OR_FETCH_FETCHED OFF)
    set(FIND_OR_FETCH_HAVE OFF)

    if(ARG_BUILD_FLAG)
        option(${ARG_BUILD_FLAG} "Enable building ${ARG_NAME} when not prebuilt" ON)
    else()
        set(ARG_BUILD_FLAG BUILD_FLAG)
        set(BUILD_FLAG ON)
    endif()

    set(${ARG_NAME}_DIR "${CMAKE_INSTALL_PREFIX}/lib/cmake/${ARG_NAME}" CACHE INTERNAL "")
    message(STATUS "Looking for a CMake configuration of ${ARG_NAME} in ${${ARG_NAME}_DIR}")
    find_package(${ARG_NAME} ${ARG_VERSION} QUIET CONFIG)

    if(${ARG_NAME}_FOUND)
        message(STATUS "Found prebuilt ${ARG_NAME} at ${${ARG_NAME}_DIR}")
        set(FIND_OR_FETCH_FOUND ON)
    elseif(${ARG_BUILD_FLAG})
        include(FetchContent)

        string(TOLOWER ${ARG_NAME} LCASE_NAME)
        string(TOUPPER  ${ARG_NAME} UCASE_NAME)
        set(LOCAL_DIR LOCAL_${UCASE_NAME}_DIR)

        set(${LOCAL_DIR}
            "${CMAKE_SOURCE_DIR}/.deps/source/${ARG_NAME}-${ARG_GIT_REF}"
            CACHE PATH "Path to the local ${ARG_NAME} directory")

        if (IS_DIRECTORY "${${LOCAL_DIR}}")
            message(STATUS "Found a local copy of ${ARG_NAME} at ${${LOCAL_DIR}}")
            fetchcontent_declare(${LCASE_NAME} URL "${${LOCAL_DIR}}")
            set(FIND_OR_FETCH_FETCHED ON)
        elseif(NOT NO_INTERNET)
            fetchcontent_declare(${LCASE_NAME}
                GIT_REPOSITORY "${ARG_GIT_URL}"
                GIT_TAG "v${ARG_GIT_TAG}"
                GIT_PROGRESS TRUE)
            set(FIND_OR_FETCH_FETCHED ON)
        endif()
    endif()

    if(FIND_OR_FETCH_FOUND OR FIND_OR_FETCH_FETCHED)
        set(FIND_OR_FETCH_HAVE ON)
    elseif(ARG_REQUIRED)
        message(STATUS "Either set ${ARG_NAME}_DIR to a directory that contains a package configuration file for ${ARG_NAME},")
        message(STATUS "or set ${LOCAL_DIR} to a directory that contains the source tree of ${ARG_NAME} ${ARG_VERSION}.")
        message(STATUS "Currently ${ARG_NAME}_DIR is ${${ARG_NAME}_DIR} and ${LOCAL_DIR} is ${${LOCAL_DIR}}.")
        message(FATAL_ERROR "${ARG_NAME} ${VERSION} is required.")
    else()
        message(STATUS "${ARG_NAME} is disabled.")
    endif()

    set(FIND_OR_FETCH_FOUND ${FIND_OR_FETCH_FOUND} PARENT_SCOPE)
    set(FIND_OR_FETCH_FETCHED ${FIND_OR_FETCH_FETCHED} PARENT_SCOPE)
    set(FIND_OR_FETCH_HAVE ${FIND_OR_FETCH_HAVE} PARENT_SCOPE)
endfunction()
