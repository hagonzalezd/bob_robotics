// C++ includes
#include <iostream>

// GeNNRobotics includes
#include "imgproc/opencv_unwrap_360.h"

int
main(int argc, char **argv)
{
    // create new unwrapper with desired params
    GeNNRobotics::ImgProc::OpenCVUnwrap360 unwrapper(cv::Size(1280, 720),
                                                     cv::Size(1280, 400),
                                                     0.45468750000000002,
                                                     0.20499999999999999,
                                                     0.087499999999999994,
                                                     0.19,
                                                     0,
                                                     false);

    // open file for writing
    const std::string fileName = "serialise_test.yaml";
    std::cout << "Writing to " << fileName << "..." << std::endl;
    cv::FileStorage fs(fileName, cv::FileStorage::WRITE);

    // write unwrap parameters to file
    fs << "unwrapper" << unwrapper;

    // close file
    fs.release();
}