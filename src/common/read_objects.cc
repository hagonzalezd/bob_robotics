// BoB robotics includes
#include "common/assert.h"
#include "common/pose.h"
#include "common/read_objects.h"

// OpenCV
#include <opencv2/opencv.hpp> // for YAML deserialisation

// Standard C++ includes
#include <vector>

namespace BoBRobotics {
/**!
 *  \brief Read a set of objects (vertices) from a YAML file
 *
 * The files are of the form generated by save_object, which can be found here:
 *      https://github.com/BrainsOnBoard/vicon_marker_tools
 */
auto readObjects(const filesystem::path &objectFilePath) {
    BOB_ASSERT(objectFilePath.exists());

    using namespace units::length;
    std::vector<std::vector<Vector2<millimeter_t>>> objects;

    cv::FileStorage fs(objectFilePath.str(), cv::FileStorage::READ);
    std::vector<double> vertex(2);
    for (auto objectNode : fs["objects"]) {
        objects.emplace_back();
        for (auto vertexNode : objectNode) {
            vertexNode >> vertex;

            // Check that we have x and y values
            BOB_ASSERT(vertex.size() == 2);

            objects.back().emplace_back(millimeter_t(vertex[0]), millimeter_t(vertex[1]));
        }
    }

    return objects;
}
} // BoBRobotics