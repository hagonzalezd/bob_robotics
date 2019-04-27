cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES remote_tank_db_creator.cc
            BOB_MODULES common hid navigation net robots robots/control vicon
                        video viz/sfml_world)
