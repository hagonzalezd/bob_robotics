cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES camera_360.cc
            BOB_MODULES common imgproc video)
