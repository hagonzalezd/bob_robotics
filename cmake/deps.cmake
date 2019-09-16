function(BoB_modules)
    foreach(module IN LISTS ARGV)
        set(module_path ${BOB_ROBOTICS_PATH}/src/${module})

        # Some (sub)modules have a slash in the name; replace with underscore
        string(REPLACE / _ module_name ${module})

        # All of our targets depend on this module
        foreach(target IN LISTS BOB_TARGETS)
            add_dependencies(${target} bob_${module_name})
        endforeach()

        # Build subdirectory
        if(NOT TARGET bob_${module_name})
            add_subdirectory(${module_path} "${BOB_DIR}/modules/${module_name}")
        endif()

        # Link against BoB module static lib + its dependencies
        BoB_add_link_libraries(bob_${module_name} ${bob_${module_name}_LIBRARIES})
        BoB_add_include_directories(${bob_${module_name}_INCLUDE_DIRS})
    endforeach()
endfunction()

function(BoB_external_libraries)
    foreach(lib IN LISTS ARGV)
        set(incpath "${BOB_ROBOTICS_PATH}/cmake/external_libs/${lib}.cmake")
        if(EXISTS "${incpath}")
            include("${incpath}")
        else()
            message(FATAL_ERROR "${lib} is not a recognised library name")
        endif()
    endforeach()
endfunction()

function(BoB_third_party)
    foreach(module IN LISTS ARGV)
        # Extra actions for third-party modules
        set(incpath "${BOB_ROBOTICS_PATH}/cmake/third_party/${module}.cmake")
        if(EXISTS "${incpath}")
            include("${incpath}")
        endif()

        if(EXISTS "${BOB_ROBOTICS_PATH}/third_party/${module}")
            # Checkout git submodules under this path
            find_package(Git REQUIRED)
            exec_or_fail(${GIT_EXECUTABLE} submodule update --init --recursive third_party/${module}
                            WORKING_DIRECTORY "${BOB_ROBOTICS_PATH}")

            # If this folder is a cmake project, then build it
            set(module_path ${BOB_ROBOTICS_PATH}/third_party/${module})
            if(EXISTS ${module_path}/CMakeLists.txt)
                add_subdirectory(${module_path} "${BOB_DIR}/third_party/${module}")
            endif()

            # Add to include path
            set(module_path ${BOB_ROBOTICS_PATH}/third_party/${module})
            include_directories(${module_path} ${module_path}/include ${${module}_INCLUDE_DIRS})

            # Link against extra libs, if needed
            BoB_add_link_libraries(${${module}_LIBRARIES})
        endif()
    endforeach()
endfunction()
