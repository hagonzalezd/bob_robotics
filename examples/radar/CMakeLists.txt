cmake_minimum_required(VERSION 3.1)
include(../../cmake/bob_robotics.cmake)
BoB_project(SOURCES radar.cc
            BOB_MODULES common hid
            PLATFORMS linux windows
	    EXTERNAL_LIBS serial)

