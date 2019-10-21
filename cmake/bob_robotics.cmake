cmake_minimum_required(VERSION 3.1)
include("${CMAKE_CURRENT_LIST_DIR}/deps.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/module.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/project.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/utils.cmake")

macro(BoB_set_options)
    # Extra compile-type options
    if(PARSED_ARGS_OPTIONS)
        foreach(option IN LISTS PARSED_ARGS_OPTIONS)
            if(${option})
                message("Option: ${option}=on")
                set(OPTION_${option} TRUE)
                add_definitions(-D${option})
            else()
                message("Option: ${option}=off")
            endif()
        endforeach()
    endif()

    # For the various USE_* C macros we want defined
    set_use_macros()
endmacro()

# Set all the USE_* macros before we do anything else
macro(set_use_macros)
    foreach(lib IN LISTS PARSED_ARGS_EXTERNAL_LIBS)
        add_use_macro(${lib})
    endforeach()
    foreach(lib IN LISTS PARSED_ARGS_THIRD_PARTY)
        add_use_macro(${lib})
    endforeach()
    foreach(module IN LISTS PARSED_ARGS_BOB_MODULES)
        if(NOT "${module}" STREQUAL "")
            # Some BoB modules have slashes in the name; replace with underscore
            string(REPLACE / _ module ${module})
            add_use_macro(BOB_${module})
        endif()
    endforeach()
endmacro()

# Sets a C++ macro, so you can have compile-time options in your header (e.g.
# some functions that will only work if your project uses OpenCV)
function(add_use_macro libname)
    string(TOUPPER "${libname}" libnameupper)
    string(REPLACE - _ libnameupper ${libnameupper})

    # Pass USE flags to parent project
    set(USE_FLAGS "${USE_FLAGS};${libnameupper}" CACHE INTERNAL "${PROJECT_NAME}: USE flags")
endfunction()

macro(BoB_init)
    # CMake defaults to 32-bit builds on Windows
    if(WIN32 AND NOT CMAKE_GENERATOR_PLATFORM)
        message(WARNING "CMAKE_GENERATOR_PLATFORM is set to x86. This is probably not what you want!")
    endif()

    # For release builds, CMake disables assertions, but a) this isn't what we
    # want and b) it will break code.
    if(MSVC)
        string(REPLACE "/DNDEBUG" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
    else()
        string(REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
    endif()

    # Don't allow in-source builds
    if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
        message(FATAL_ERROR "In-source builds not allowed.
        Please make a new directory (called a build directory) and run CMake from there.
        You may need to remove CMakeCache.txt." )
    endif()

    # If this var is defined then this project is being included in another build
    if(NOT DEFINED BOB_DIR)
        if(WIN32)
            set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${BOB_ROBOTICS_PATH}/bin)
            set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${BOB_ROBOTICS_PATH}/bin)
            set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BOB_ROBOTICS_PATH}/bin)
        else()
            set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
        endif()

        # Folder to build BoB modules + third-party modules
        set(BOB_DIR "${CMAKE_CURRENT_BINARY_DIR}/BoB")
    endif()

    # Use vcpkg on Windows
    if(WIN32)
        # Use vcpkg's cmake toolchain
        if(DEFINED ENV{VCPKG_ROOT})
            if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
                set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
                    CACHE STRING "")
            endif()
        else()
            message(FATAL_ERROR "The environment VCPKG_ROOT must be set on Windows")
        endif()

        # The vcpkg toolchain in theory should do something like this already, but
        # if we don't do this, then cmake can't find any of vcpkg's packages
        file(GLOB children "$ENV{VCPKG_ROOT}/installed/${CMAKE_GENERATOR_PLATFORM}-windows/share/*")
        foreach(child IN LISTS children)
            if(IS_DIRECTORY "${child}")
                list(APPEND CMAKE_PREFIX_PATH "${child}")
            endif()
        endforeach()

        # Suppress warnings about std::getenv being insecure
        add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    endif()

    # Assume we always need plog
    BoB_third_party(plog)

    # Default include paths
    include_directories(${BOB_ROBOTICS_PATH}
                        ${BOB_ROBOTICS_PATH}/include)

    # Disable some of the units types in units.h for faster compilation
    add_definitions(
        -DDISABLE_PREDEFINED_UNITS
        -DENABLE_PREDEFINED_LENGTH_UNITS
        -DENABLE_PREDEFINED_TIME_UNITS
        -DENABLE_PREDEFINED_ANGLE_UNITS
        -DENABLE_PREDEFINED_VELOCITY_UNITS
        -DENABLE_PREDEFINED_ANGULAR_VELOCITY_UNITS
    )

    # Look for additional CMake packages in the current folder
    set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})
endmacro()

macro(always_included_packages)
    # Assume we always want threading
    find_package(Threads REQUIRED)

    # Annoyingly, these packages export a target rather than simply variables
    # with the include path and link flags and it seems that this target isn't
    # "passed up" by add_subdirectory(), so we always include these packages on
    # the off-chance we need them.
    if(NOT "${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" AND NOT TARGET OpenMP::OpenMP_CXX)
        find_package(OpenMP QUIET)
    endif()
    if(NOT TARGET GLEW::GLEW)
        find_package(GLEW QUIET)
    endif()

    # On Unix we use pkg-config to find SDL2 or Eigen, because the CMake
    # packages may not be present
    if(NOT UNIX)
        if(NOT TARGET SDL2::SDL2)
            find_package(SDL2)
        endif()
        if(NOT TARGET Eigen3::Eigen)
            find_package(Eigen3 QUIET)
        endif()
    endif()
endmacro()

macro(BoB_build)
    # Don't build i2c code if NO_I2C environment variable is set
    if(NOT I2C_MESSAGE_DISPLAYED AND (NO_I2C OR (NOT "$ENV{NO_I2C}" STREQUAL 0 AND NOT "$ENV{NO_I2C}" STREQUAL "")))
        set(I2C_MESSAGE_DISPLAYED TRUE)
        message("NO_I2C is set: not building i2c code")
        set(NO_I2C TRUE)
        add_definitions(-DNO_I2C)
    endif()

    # Add macro so that programs know where the root folder is for e.g. loading
    # resources
    add_definitions(-DBOB_ROBOTICS_PATH="${BOB_ROBOTICS_PATH}")

    # Default to building release type
    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
    endif()
    message("Build type: ${CMAKE_BUILD_TYPE}")

    if(NOT WIN32)
        # Use ccache if present to speed up repeat builds
        find_program(CCACHE_FOUND ccache)
        if(CCACHE_FOUND)
            set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
            set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
        else()
            message(WARNING "ccache not found. Install for faster repeat builds.")
        endif()
    endif()

    # Set DEBUG macro when compiling in debug mode
    if(${CMAKE_BUILD_TYPE} STREQUAL Debug)
        add_definitions(-DDEBUG)
    endif()

    # Flags for gcc and clang
    if (NOT GNU_TYPE_COMPILER AND ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU" OR "${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang"))
        set(GNU_TYPE_COMPILER TRUE)

        # Default to building with -march=native
        if(NOT DEFINED ENV{ARCH})
            set(ENV{ARCH} native)
        endif()

        # Enable warnings and set architecture
        add_compile_flags("-Wall -Wpedantic -Wextra -march=$ENV{ARCH}")

        # Gcc has an annoying feature where you can mark functions with
        # __attribute__((warn_unused_result)) and then the calling code *has*
        # to do something with the result and can't ignore it; hacks such as
        # (void) annoyingFunction() don't work either. We're mostly
        # seeing this warning for calls to std::system() (in our code and third-
        # party code), but in those cases we generally really don't care about
        # the return value. So let's just disable it globally to save faffing
        # around.
        add_compile_flags(-Wno-unused-result)

        # I'm getting warnings based for code in the Eigen headers, so let's
        # just disable it. I tried setting this flag only when we're actually
        # using Eigen, but that didn't seem to work, and it seems pretty
        # harmless, so it's probably fine to just disable it globally.
        #          - AD
        #
        # Eigen version: 3.3.7
        # gcc version:   9.1.0
        if (CMAKE_COMPILER_IS_GNUCC AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 9.0)
            add_compile_flags(-Wno-deprecated-copy)
        endif()

        # Disable optimisation for debug builds
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

        # If we don't do this, I get linker errors on the BrickPi for the net
        # module
        set(CMAKE_EXE_LINKER_FLAGS "-Wl,--allow-multiple-definition")
    endif()

    # Use C++14. On Ubuntu 16.04, seemingly setting CMAKE_CXX_STANDARD doesn't
    # work, so add the compiler flag manually.
    #
    # Conversely, only setting the compiler flag means that the surveyor example
    # mysteriously gets linker errors on Ubuntu 18.04 and my Arch Linux machine.
    #       - AD
    if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        add_compile_flags(-std=c++14)
    endif()
    set(CMAKE_CXX_STANDARD 14)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)

    # Irritatingly, neither GCC nor Clang produce nice ANSI-coloured output if they detect
    # that output "isn't a terminal" - which seems to include whatever pipe-magick cmake includes.
    # https://medium.com/@alasher/colored-c-compiler-output-with-ninja-clang-gcc-10bfe7f2b949
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
       add_compile_flags(-fdiagnostics-color=always)
    elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
       add_compile_flags(-fcolor-diagnostics)
    endif ()

    # Different Jetson devices have different user-facing I2C interfaces
    # so read the chip ID and add preprocessor macro
    if(EXISTS /sys/module/tegra_fuse/parameters/tegra_chip_id)
        file(READ /sys/module/tegra_fuse/parameters/tegra_chip_id TEGRA_CHIP_ID)
        add_definitions(-DTEGRA_CHIP_ID=${TEGRA_CHIP_ID})
        message("Tegra chip id: ${TEGRA_CHIP_ID}")
    endif()

    # Set include dirs and link libraries for this module/project
    always_included_packages()
    BoB_external_libraries(${PARSED_ARGS_EXTERNAL_LIBS})
    BoB_third_party(${PARSED_ARGS_THIRD_PARTY})
    BoB_modules(${PARSED_ARGS_BOB_MODULES})

    # Link threading lib
    BoB_add_link_libraries(${CMAKE_THREAD_LIBS_INIT})

    # Clang needs to be linked against libm and libstdc++ explicitly
    if("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
        BoB_add_link_libraries(m stdc++)
    endif()

    # The list of linked libraries can end up very long with lots of duplicate
    # entries and this can break ld, so remove them. We remove from the start,
    # so that dependencies will always (I think!) be in the right order.
    list(REVERSE ${PROJECT_NAME}_LIBRARIES)
    list(REMOVE_DUPLICATES ${PROJECT_NAME}_LIBRARIES)
    list(REVERSE ${PROJECT_NAME}_LIBRARIES)

    # Link all targets against the libraries
    foreach(target IN LISTS BOB_TARGETS)
        target_link_libraries(${target} ${${PROJECT_NAME}_LIBRARIES})
    endforeach()

    # Set USE flags; includes flags from child projects
    list(REMOVE_DUPLICATES USE_FLAGS)
    message("USE flags for ${PROJECT_NAME}")
    foreach(flag IN LISTS USE_FLAGS)
        if(NOT "${flag}" STREQUAL "")
            add_definitions(-DUSE_${flag})
            message("    -- USE_${flag}")
        endif()
    endforeach(flag IN LISTS USE_FLAGS)
endmacro()

# Set output directories for libs and executables
get_filename_component(BOB_ROBOTICS_PATH .. ABSOLUTE BASE_DIR "${CMAKE_CURRENT_LIST_DIR}")
