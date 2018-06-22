#pragma once

// C++ includes
#include <iostream>
#include <string>
#include <vector>

// BoB robotics includes
#include "third_party/path.h"

// local includes
#include "unwrap.h"

/*
 * ffmpeg is used for copying the audio stream from the original to the
 * unwrapped video
 */
#ifndef FFMPEG_PATH
#define FFMPEG_PATH "/usr/bin/ffmpeg"
#endif


/* unwrap a JPEG file */
void processjpeg(const char *filepathRaw)
{
    filesystem::path filepath(filepathRaw);

    // read image into memory
    cv::Mat im = cv::imread(filepath.str(), CV_LOAD_IMAGE_COLOR);

    // matrix to store unwrapped image
    cv::Mat imunwrap(unwrap_height, unwrap_width, im.type());

    // matrices to store maps for unwrapping
    cv::Mat map_x(unwrap_height, unwrap_width, CV_32FC1);
    cv::Mat map_y(unwrap_height, unwrap_width, CV_32FC1);

    // perform unwrapping
    if (!processframe(filepath.str().c_str(), imunwrap, im, map_x, map_y)) {
        return;
    }

    // save file
    filesystem::path outfilename = filepath.parent_path() / ("unwrapped_" + filepath.filename());
    std::cout << "Saving image to " << outfilename.str() << "..." << std::endl;
    std::vector<int> params;
    params.push_back(CV_IMWRITE_JPEG_QUALITY);
    params.push_back(100);
    cv::imwrite(outfilename.str(), imunwrap, params);
}

bool copysound = true;

/* unwrap an MP4 video */
void processmp4(const char *filepathRaw)
{
    filesystem::path filepath(filepathRaw);

    // open video file
    cv::VideoCapture cap(filepath.str());

    // read in first frame
    cv::Mat fr;
    cap >> fr;

    // matrices to store maps for unwrapping
    cv::Mat map_x(unwrap_height, unwrap_width, CV_32FC1);
    cv::Mat map_y(unwrap_height, unwrap_width, CV_32FC1);

    // matrix to store unwrapped image
    cv::Mat imunwrap(unwrap_height, unwrap_width, fr.type());

    // try unwrapping first frame and return if fail
    if (!processframe(filepath.str().c_str(), imunwrap, fr, map_x, map_y)) {
        cap.release();
        return;
    }

    // final filename for unwrapped video
    filesystem::path outfilename = filepath.parent_path() / ("unwrapped_" + filepath.filename());

    // temporary file name to which we write initially
    filesystem::path tempfilename = copysound ? filepath.parent_path() / ".TEMP.MP4": outfilename;

    // start writing to file
    std::cout << "Saving video to " << outfilename << "..." << std::endl;
    cv::VideoWriter writer(
            tempfilename.str(), 0x21, cap.get(CV_CAP_PROP_FPS), imunwrap.size());
    if (!writer.isOpened()) {
        std::cerr << "Error: Could not open file for writing" << std::endl;
        return;
    }

    // write first unwrapped frame
    writer.write(imunwrap);

    // unwrap successive frames and write to file
    for (;;) {
        cap >> fr;
        if (fr.empty()) {
            break;
        }

        unwrap(imunwrap, fr, map_x, map_y);
        writer.write(imunwrap);
    }

    // dispose of writer and reader when finished
    cap.release();
    writer.release();

    if (copysound) {
        // attempt to copy audio stream from original to new file with ffmpeg
        int stat = system((FFMPEG_PATH " -y -i \"" + tempfilename.str() + "\" -i \"" +
                           filepath.str() +
                           "\" -map 0:v -map 1:a -c copy -shortest \"" +
                           outfilename.str() + "\" >/dev/null 2>&1")
                                  .c_str());
        if (stat != 0) {
            std::cerr << "Error (" << stat
                 << ") occurred while copying audio with ffmpeg" << std::endl;

            /*
             * just rename the temporary file to the output file, so there won't
             * be audio but at least there'll be video
             */
            rename(tempfilename.str().c_str(), outfilename.str().c_str());
        } else {
            // successfully copied audio, so delete temporary file
            remove(tempfilename.str().c_str());
        }
    }
}
