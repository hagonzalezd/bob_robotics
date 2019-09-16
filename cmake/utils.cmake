macro(add_compile_flags EXTRA_ARGS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_ARGS}")
endmacro()

macro(add_linker_flags EXTRA_ARGS)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${EXTRA_ARGS}")
endmacro()

function(check_platform)
    # If it's an empty list, return
    if(${ARGC} EQUAL 0)
        return()
    endif()

    # Check the platforms are valid
    foreach(platform IN LISTS ARGV)
        if(NOT "${platform}" STREQUAL unix
           AND NOT "${platform}" STREQUAL linux
           AND NOT "${platform}" STREQUAL windows
           AND NOT "${platform}" STREQUAL all)
            message(FATAL_ERROR "Bad platform: ${platform}. Possible platforms are unix, linux, windows or all.")
        endif()
    endforeach()

    # Check for platform all
    list(FIND ARGV all index)
    if(${index} GREATER -1)
        return()
    endif()

    # Check for platform unix
    if(UNIX)
        list(FIND ARGV unix index)
        if(${index} GREATER -1)
            return()
        endif()

        # Check for platform linux
        if("${CMAKE_SYSTEM_NAME}" STREQUAL Linux)
            list(FIND ARGV linux index)
            if(${index} GREATER -1)
                return()
            endif()
        endif()
    endif()

    # Check for platform windows
    if(WIN32)
        list(FIND ARGV windows index)
        if(${index} GREATER -1)
            return()
        endif()
    endif()

    # Otherwise we haven't matched any platforms
    message(FATAL_ERROR "This machine is not one of the supported platforms for this project (${ARGV}).")
endfunction()

macro(BoB_add_link_libraries)
    set(${PROJECT_NAME}_LIBRARIES "${${PROJECT_NAME}_LIBRARIES};${ARGV}"
        CACHE INTERNAL "${PROJECT_NAME}: Libraries" FORCE)
endmacro()

function(BoB_deprecated WHAT ALTERNATIVE)
    message(WARNING "!!!!! WARNING: Use of ${WHAT} in BoB robotics code is deprecated and will be removed in future. Use ${ALTERNATIVE} instead. !!!!!")
endfunction()

function(BoB_add_include_directories)
    # Sometimes we get newline characters in an *_INCLUDE_DIRS variable (e.g.
    # with the OpenCV package) and this breaks CMake
    string(REPLACE "\n" " " ARGV "${ARGV}")

    # Include directory locally...
    include_directories(${ARGV})

    # ...and export to parent project
    set(${PROJECT_NAME}_INCLUDE_DIRS "${${PROJECT_NAME}_INCLUDE_DIRS};${ARGV}"
        CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)
    list(REMOVE_DUPLICATES ${PROJECT_NAME}_INCLUDE_DIRS)
endfunction()

function(BoB_find_package)
    find_package(${ARGV})
    BoB_add_include_directories(${${ARGV0}_INCLUDE_DIRS})
    BoB_add_link_libraries(${${ARGV0}_LIBS} ${${ARGV0}_LIBRARIES})
endfunction()

function(BoB_add_pkg_config_libraries)
    find_package(PkgConfig)
    foreach(lib IN LISTS ARGV)
        pkg_check_modules(${lib} REQUIRED ${lib})
        BoB_add_include_directories(${${lib}_INCLUDE_DIRS})
        BoB_add_link_libraries(${${lib}_LIBRARIES})
    endforeach()
endfunction()

macro(exec_or_fail)
    execute_process(COMMAND ${ARGV} RESULT_VARIABLE rv OUTPUT_VARIABLE SHELL_OUTPUT)
    if(NOT ${rv} EQUAL 0)
        message(FATAL_ERROR "Error while executing: ${ARGV}")
    endif()
endmacro()
