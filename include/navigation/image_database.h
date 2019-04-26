#pragma once

// BoB robotics includes
#include "common/assert.h"
#include "common/pose.h"
#include "video/input.h"

// Third-party includes
#include "third_party/path.h"
#include "third_party/units.h"

// OpenCV includes
#include <opencv2/opencv.hpp>

// Standard C++ includes
#include <array>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace BoBRobotics {
namespace Navigation {
using namespace units::literals;

//------------------------------------------------------------------------
// BoBRobotics::Navigation::Range
//------------------------------------------------------------------------
//! A range of values in millimetres
struct Range
{
    using millimeter_t = units::length::millimeter_t;
    const millimeter_t begin, end, separation;

    constexpr Range(const std::pair<millimeter_t, millimeter_t> beginAndEnd,
                    const millimeter_t separation)
    : begin(beginAndEnd.first)
    , end(beginAndEnd.second)
    , separation(separation)
    {
        if (begin == end) {
            BOB_ASSERT(separation == 0_mm);
        } else {
            BOB_ASSERT(begin < end);
            BOB_ASSERT(separation > 0_mm);
        }
    }

    constexpr Range(const millimeter_t value)
      : Range({ value, value }, 0_mm)
    {}

    size_t size() const;
};

//------------------------------------------------------------------------
// BoBRobotics::Navigation::ImageDatabase
//------------------------------------------------------------------------
//! An interface for reading from and writing to folders of images
class ImageDatabase
{
    using degree_t = units::angle::degree_t;
    using millimeter_t = units::length::millimeter_t;

public:
    //! The metadata for an entry in an ImageDatabase
    struct Entry
    {
        Vector3<millimeter_t> position;
        degree_t heading;
        filesystem::path path;
        std::array<size_t, 3> gridPosition; //! For grid-type databases, indicates the x,y,z grid position

        cv::Mat load() const;
        cv::Mat loadGreyscale() const;
    };

    //! Base class for GridRecorder and RouteRecorder
    class Recorder {
    public:
        ~Recorder();

        //! Get an object for writing additional metadata to
        cv::FileStorage &getMetadataWriter();

        //! Write extra metadata into the YAML file
        void addMetadata(const Video::Input &video, bool needsUnwrapping, bool isGreyscale);

        //! Don't save new metadata when this class is destroyed
        void abortSave();

        //! Save new metadata
        void save();

        //! Current number of *new* entries for the ImageDatabase
        size_t size() const;

        //! Get the format in which images will be saved
        std::string getImageFormat() const;

    private:
        ImageDatabase &m_ImageDatabase;
        const std::string m_ImageFormat;
        bool m_Recording;
        std::vector<Entry> m_NewEntries;

    protected:
        cv::FileStorage m_YAML;

        Recorder(ImageDatabase &imageDatabase,
                 const bool isRoute,
                 const std::string &imageFormat);

        void addEntry(const std::string &filename,
                      const cv::Mat &image,
                      const Vector3<millimeter_t> &position,
                      const degree_t heading,
                      const std::array<size_t, 3> &gridPosition = { 0, 0, 0 });
    };

    struct Noop
    {
        bool operator()()
        {
            return true;
        }
    };

    //! For recording a grid of images at a fixed heading
    template<bool alternateX = false, bool alternateY = false>
    class GridRecorder : public Recorder {
    public:
        GridRecorder(ImageDatabase &imageDatabase, const Range &xrange, const Range &yrange,
                     const Range &zrange = Range(0_mm), degree_t heading = 0_deg,
                     const std::string &imageFormat = "png")
          : Recorder(imageDatabase, false, imageFormat)
          , m_Heading(heading)
          , m_Begin(xrange.begin, yrange.begin, zrange.begin)
          , m_Separation(xrange.separation, yrange.separation, zrange.separation)
          , m_Size({ xrange.size(), yrange.size(), zrange.size() })
          , m_Current(0)
        {
            BOB_ASSERT(!imageDatabase.isRoute());

            // Save some extra, grid-specific metadata
            m_YAML << "grid" << "{"
                   << "beginAtMM" << "[:" << m_Begin[0]() << m_Begin[1]() << m_Begin[2]() << "]"
                   << "separationMM" << "[:" << m_Separation[0]() << m_Separation[1]() << m_Separation[2]() << "]"
                   << "size" << "[:" << (int) m_Size[0] << (int) m_Size[1] << (int) m_Size[2] << "]"
                   << "}";

            m_GridPositions.reserve(maximumSize());
            bool xDecreasing = false, yDecreasing = false;
            const auto xloop = [&](size_t y, size_t z)
            {
                if (xDecreasing) {
                    for (int x = sizeX() - 1; x >= 0; x--) {
                        const std::array<size_t, 3> pos = { (size_t) x, (size_t) y, z };
                        m_GridPositions.push_back(pos);
                    }
                } else {
                    for (int x = 0; x < (int) sizeX(); x++) {
                        const std::array<size_t, 3> pos = { (size_t) x, (size_t) y, z };
                        m_GridPositions.push_back(pos);
                    }
                }
                if (alternateX) {
                    xDecreasing = !xDecreasing;
                }
            };
            for (size_t z = 0; z < sizeZ(); z++) {
                if (yDecreasing) {
                    for (int y = sizeY() - 1; y >= 0; y--) {
                        xloop(y, z);
                    }
                } else {
                    for (int y = 0; y < (int) sizeY(); y++) {
                        xloop(y, z);
                    }
                }
                if (alternateY) {
                    yDecreasing = !yDecreasing;
                }
            }
        }

        //! Get the physical position represented by grid coordinates
        auto getPosition(const std::array<size_t, 3> &gridPosition) const
        {
            BOB_ASSERT(gridPosition[0] < m_Size[0] && gridPosition[1] < m_Size[1] && gridPosition[2] < m_Size[2]);
            Vector3<millimeter_t> position;
            for (size_t i = 0; i < position.size(); i++) {
                position[i] = (m_Separation[i] * gridPosition[i]) + m_Begin[i];
            }
            return position;
        }

        //! Get a vector of all possible positions for this grid
        auto getPositions() const
        {
            std::vector<Vector3<millimeter_t>> positions;
            positions.reserve(maximumSize());

            for (auto &gridPosition : m_GridPositions) {
                positions.emplace_back(getPosition(gridPosition));
            }
            return positions;
        }

        const auto &getGridPositions() const
        {
             return m_GridPositions;
        }

        auto getPosition() const
        {
            return getPosition(m_GridPositions[m_Current]);
        }

        //! Save a new image into the database
        bool record(const cv::Mat &image)
        {
            BOB_ASSERT(m_Current < m_GridPositions.size());
            record(m_GridPositions[m_Current], image);
            return ++m_Current < m_GridPositions.size();
        }

        //! Save a new image into the database at the specified coordinates
        void record(const std::array<size_t, 3> &gridPosition, const cv::Mat &image)
        {
            const auto position = getPosition(gridPosition);
            const std::string filename = ImageDatabase::getFilename(position, getImageFormat());
            addEntry(filename, image, position, m_Heading, { gridPosition[0], gridPosition[1], gridPosition[2] });
        }

        template<class AgentType, class Func = Noop>
        bool run(AgentType &agent, Video::Input &video, Func extraCalls = Noop())
        {
            BOB_ASSERT(m_Current == 0);
            return runAtPositions(agent, video, m_GridPositions, extraCalls);
        }

        template<class AgentType, class GridPositionsType, class Func = Noop>
        bool runAtPositions(AgentType &agent, Video::Input &video, const GridPositionsType &gridPositions, Func extraCalls = Noop())
        {
            BOB_ASSERT(gridPositions.size() > 0);

            cv::Mat fr;
            for (auto &gridPosition : gridPositions) {
                const auto pos = getPosition(gridPosition);
                std::cout << "Moving to: " << pos << std::endl;
                if (!agent.moveToSync(pos, extraCalls)) {
                    return false;
                }

                video.readFrameSync(fr);
                record(gridPosition, fr);
            }
            return true;
        }

        size_t maximumSize() const { return sizeX() * sizeY() * sizeZ(); }
        size_t sizeX() const { return m_Size[0]; }
        size_t sizeY() const { return m_Size[1]; }
        size_t sizeZ() const { return m_Size[2]; }

    private:
        const degree_t m_Heading;
        const Vector3<millimeter_t> m_Begin, m_Separation;
        std::vector<std::array<size_t, 3>> m_GridPositions;
        const std::array<size_t, 3> m_Size;
        size_t m_Current;
    };

    //! For saving images recorded along a route
    class RouteRecorder : public Recorder {
    public:
        RouteRecorder(ImageDatabase &imageDatabase, const std::string &imageFormat = "png");

        //! Save a new image taken at the specified pose
        void record(const Vector3<millimeter_t> &position, degree_t heading, const cv::Mat &image);

        template<class AgentType, class ContainerOfPoses, class Func = Noop>
        bool run(AgentType &agent, Video::Input &video, const ContainerOfPoses &poses, Func extraCalls = Noop())
        {
            cv::Mat fr;
            for (auto &pose : poses) {
                if (!agent.moveToSync(pose, extraCalls())) {
                    return false;
                }
                video.readFrameSync(fr);
                record(pose.position(), pose.yaw(), fr);
            }
            return true;
        }
    };

    ImageDatabase(const char *databasePath, bool overwrite = false);
    ImageDatabase(const std::string &databasePath, bool overwrite = false);
    ImageDatabase(const filesystem::path &databasePath, bool overwrite = false);

    //! Delete all files in this image database
    void deleteAll() const;

    //! Get the path of the directory corresponding to this ImageDatabase
    const filesystem::path &getPath() const;

    //! Get one Entry from the database
    const Entry &operator[](size_t i) const;

    //! Start iterator for the database entries
    std::vector<Entry>::const_iterator begin() const;

    //! End iterator for the database entries
    std::vector<Entry>::const_iterator end() const;

    //! Number of entries in this database
    size_t size() const;

    //! Check if there are any entries in this database
    bool empty() const;

    //! Check if the database is non-empty and a route-type database
    bool isRoute() const;

    //! Check if the database is non-empty and a grid-type database
    bool isGrid() const;

    //! Load all of the images in this database into memory and return
    std::vector<cv::Mat> getImages() const;

    //! Load all of the images in this database into the specified std::vector<>
    void getImages(std::vector<cv::Mat> &images) const;

    //! Access the metadata for this database via OpenCV's persistence API
    cv::FileNode getMetadata() const;

    //! Get the (directory) name of this database
    std::string getName() const;

    //! Start recording a grid of images
    template<bool alternateX = false, bool alternateY = false>
    auto getGridRecorder(const Range &xrange, const Range &yrange,
                         const Range &zrange = Range(0_mm),
                         degree_t heading = 0_deg,
                         const std::string &imageFormat = "png")
    {
        return GridRecorder<alternateX, alternateY>(*this, xrange, yrange, zrange, heading, imageFormat);
    }

    //! Start recording a route
    RouteRecorder getRouteRecorder(const std::string &imageFormat = "png");

    //! Get the resolution of saved images
    cv::Size getResolution() const;

    //! Check if this database has any saved metadata (yet)
    bool hasMetadata() const;

    /**!
     *  \brief Unwrap all the panoramic images in this database into a new
     *         folder, creating a new database.
     */
    void unwrap(const filesystem::path &destination, const cv::Size &unwrapRes);

    //! Get a filename for a route-type database
    static std::string getFilename(const size_t routeIndex,
                                   const std::string &imageFormat = "png");

    //! Get a filename for a grid-type database
    static std::string getFilename(const Vector3<millimeter_t> &position,
                                   const std::string &imageFormat = "png");

private:
    const filesystem::path m_Path;
    std::vector<Entry> m_Entries;
    std::unique_ptr<cv::FileStorage> m_MetadataYAML;
    cv::Size m_Resolution;
    bool m_IsRoute;
    static constexpr const char *MetadataFilename = "database_metadata.yaml";
    static constexpr const char *EntriesFilename = "database_entries.csv";

    void loadMetadata();
    void writeImage(const std::string &filename, const cv::Mat &image) const;
    void addNewEntries(std::vector<Entry> &newEntries);
    void writeEntry(std::ofstream &os, const Entry &e);
}; // ImageDatabase
} // Navigation
} // BoB robotics
