# Build a "project" in the current folder (e.g. example, etc.). Each *.cc file
# found is compiled into a separate executable.
macro(BoB_project)
    BoB_init()

    # Parse input args
    include(CMakeParseArguments)
    cmake_parse_arguments(PARSED_ARGS
                          "GENN_CPU_ONLY"
                          "EXECUTABLE;GENN_MODEL;GAZEBO_PLUGIN"
                          "SOURCES;BOB_MODULES;EXTERNAL_LIBS;THIRD_PARTY;PLATFORMS;OPTIONS"
                          "${ARGV}")
    BoB_set_options()

    if(NOT PARSED_ARGS_SOURCES AND NOT PARSED_ARGS_GAZEBO_PLUGIN)
        message(FATAL_ERROR "SOURCES not defined for BoB project")
    endif()

    # Check we're on a supported platform
    check_platform(${PARSED_ARGS_PLATFORMS})

    if(PARSED_ARGS_EXECUTABLE)
        set(NAME ${PARSED_ARGS_EXECUTABLE})
    else()
        # Use current folder as project name
        get_filename_component(NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
    endif()
    project(${NAME})

    # Include local *.h files in project. We don't strictly need to do this, but
    # if we don't then they won't be included in generated Visual Studio
    # projects.
    file(GLOB H_FILES "*.h")

    if(PARSED_ARGS_EXECUTABLE)
        # Build a single executable from these source files
        add_executable(${NAME} "${PARSED_ARGS_SOURCES}" "${H_FILES}")
        set(BOB_TARGETS ${NAME})
    else()
        # Build each *.cc file as a separate executable
        foreach(file IN LISTS PARSED_ARGS_SOURCES)
            get_filename_component(shortname ${file} NAME)
            string(REGEX REPLACE "\\.[^.]*$" "" target ${shortname})
            add_executable(${target} "${file}" "${H_FILES}")
            list(APPEND BOB_TARGETS ${target})
        endforeach()
    endif()

    if(PARSED_ARGS_GAZEBO_PLUGIN)
        get_filename_component(shortname ${PARSED_ARGS_GAZEBO_PLUGIN} NAME)
        string(REGEX REPLACE "\\.[^.]*$" "" target ${shortname})

        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

        # I'm sometimes getting linker errors when ld is linking against the
        # static libs for BoB modules (because Gazebo plugins, as shared libs,
        # are PIC, but the static libs seem not to be). So let's just compile
        # everything as PIC.
        if(GNU_TYPE_COMPILER)
            add_definitions(-fPIC)
        endif()

        # Gazebo plugins are shared libraries
        add_library(${target} SHARED ${PARSED_ARGS_GAZEBO_PLUGIN})
        list(APPEND BOB_TARGETS ${target})

        # We need to link against Gazebo libs
        BoB_external_libraries(gazebo)
    endif()

    # If this project includes a GeNN model...
    if(PARSED_ARGS_GENN_MODEL)
        get_filename_component(genn_model_name "${CMAKE_CURRENT_SOURCE_DIR}" NAME)
        set(genn_model_dir "${CMAKE_CURRENT_BINARY_DIR}/${genn_model_name}_CODE")
        set(genn_model_src "${CMAKE_CURRENT_SOURCE_DIR}/${PARSED_ARGS_GENN_MODEL}")
        set(genn_model_dest "${genn_model_dir}/runner.cc")

        if(NOT GENN_CPU_ONLY)
            if(DEFINED ENV{CPU_ONLY} AND NOT $ENV{CPU_ONLY} STREQUAL 0)
                set(GENN_CPU_ONLY TRUE)
            else()
                set(GENN_CPU_ONLY ${PARSED_ARGS_GENN_CPU_ONLY})
            endif()
        endif(NOT GENN_CPU_ONLY)
        if(GENN_CPU_ONLY)
            message("Building GeNN model for CPU only")
            add_definitions(-DCPU_ONLY)
            set(CPU_FLAG -c)
        else()
            message("Building GeNN model with CUDA")
        endif()

        # Custom command to generate source code with GeNN
        add_custom_command(PRE_BUILD
                           OUTPUT ${genn_model_dest}
                           DEPENDS ${genn_model_src}
                           COMMAND genn-buildmodel.sh
                                   ${genn_model_src}
                                   ${CPU_FLAG}
                                   -i ${BOB_ROBOTICS_PATH}:${BOB_ROBOTICS_PATH}/include
                           COMMENT "Generating source code with GeNN")

        # Custom command to generate librunner.so
        add_custom_command(PRE_BUILD
                           OUTPUT ${genn_model_dir}/librunner.so
                           DEPENDS ${genn_model_dest}
                           COMMAND make -C "${genn_model_dir}")

        add_custom_target(${PROJECT_NAME}_genn_model ALL DEPENDS ${genn_model_dir}/librunner.so)

        # Our targets depend on librunner.so
        BoB_add_include_directories(/usr/include/genn)
        BoB_add_link_libraries(${genn_model_dir}/librunner.so)
        foreach(target IN LISTS BOB_TARGETS)
            add_dependencies(${target} ${PROJECT_NAME}_genn_model)
        endforeach()

        # So code can access headers in the *_CODE folder
        BoB_add_include_directories(${CMAKE_CURRENT_BINARY_DIR})
    endif()

    # Allow users to choose the type of tank robot to use with TANK_TYPE env var
    # or CMake param (defaults to Norbot)
    if(NOT TANK_TYPE)
        if(NOT "$ENV{TANK_TYPE}" STREQUAL "")
            set(TANK_TYPE $ENV{TANK_TYPE})
        else()
            set(TANK_TYPE Norbot)
        endif()
    endif()
    add_definitions(-DTANK_TYPE=${TANK_TYPE} -DTANK_TYPE_${TANK_TYPE})
    message("Default tank robot type (if used): ${TANK_TYPE}")

    # For EV3 (Lego) robots, we need an extra module
    if(${TANK_TYPE} STREQUAL EV3)
        list(APPEND PARSED_ARGS_BOB_MODULES robots/ev3)
    endif()

    # Do linking etc.
    BoB_build()

    # Copy all DLLs over from vcpkg dir. We don't necessarily need all of them,
    # but it would be a hassle to figure out which ones we need.
    if(WIN32)
        file(GLOB dll_files "$ENV{VCPKG_ROOT}/installed/${CMAKE_GENERATOR_PLATFORM}-windows/bin/*.dll")
        foreach(file IN LISTS dll_files)
            get_filename_component(filename "${file}" NAME)
            if(NOT EXISTS "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${filename}")
                message("Copying ${filename} to ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}...")
                file(COPY "${file}"
                     DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
            endif()
        endforeach()

        link_directories("${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
    endif()
endmacro()
