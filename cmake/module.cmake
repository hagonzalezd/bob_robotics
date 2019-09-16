# Build a module from sources in current folder. All *.cc files are compiled
# into a static library.
macro(BoB_module)
    BoB_module_custom(${ARGN})
    BoB_build()
endmacro()

# Build a module with extra libraries etc. Currently used by robots/bebop
# module because the stock BoB_module() isn't flexible enough.
macro(BoB_module_custom)
    BoB_init()

    include(CMakeParseArguments)
    cmake_parse_arguments(PARSED_ARGS
                          ""
                          ""
                          "SOURCES;BOB_MODULES;OPTIONAL_BOB_MODULES;EXTERNAL_LIBS;THIRD_PARTY;PLATFORMS;OPTIONS"
                          "${ARGV}")
    if(NOT PARSED_ARGS_SOURCES)
        message(FATAL_ERROR "SOURCES not defined for BoB module")
    endif()

    # Make sure to also build optional module dependencies if we're building all
    # examples, otherwise we'll get missing symbols
    if(BUILD_ALL)
        set(PARSED_ARGS_BOB_MODULES "${PARSED_ARGS_BOB_MODULES};${PARSED_ARGS_OPTIONAL_BOB_MODULES}")
    endif()
    BoB_set_options()

    # Check we're on a supported platform
    check_platform(${PARSED_ARGS_PLATFORMS})

    # Module name is based on path relative to src/
    file(RELATIVE_PATH NAME "${BOB_ROBOTICS_PATH}/src" "${CMAKE_CURRENT_SOURCE_DIR}")
    set(BOB_TARGETS bob_${NAME})
    string(REPLACE / _ BOB_TARGETS ${BOB_TARGETS})
    project(${BOB_TARGETS})

    file(GLOB H_FILES "${BOB_ROBOTICS_PATH}/include/${NAME}/*.h")
    add_library(${BOB_TARGETS} STATIC ${PARSED_ARGS_SOURCES} ${H_FILES})
    set_target_properties(${BOB_TARGETS} PROPERTIES PREFIX ./lib)
    add_definitions(-DNO_HEADER_DEFINITIONS)
endmacro()
