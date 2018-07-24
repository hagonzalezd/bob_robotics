#pragma once

// Standard C++ includes
#include <array>
#include <set>
#include <string>
#include <vector>

// OpenGL includes
#include <GL/glew.h>
#include <GL/glu.h>

// Third-party includes
#include "../third_party/units.h"

//----------------------------------------------------------------------------
// BoBRobotics::AntWorld::RouteArdin
//----------------------------------------------------------------------------
// Class for reading ant routes exported by Matlab, performing 'straightening'
// Process from original matlab code and rendering them in ant world
namespace BoBRobotics
{
namespace AntWorld
{
using namespace units::angle;
using namespace units::length;
class RouteArdin
{
public:
    RouteArdin(float arrowLength, unsigned int maxRouteEntries);
    RouteArdin(float arrowLength, unsigned int maxRouteEntries, const std::string &filename, bool realign = true);
    ~RouteArdin();

    //------------------------------------------------------------------------
    // Public API
    //------------------------------------------------------------------------
    bool load(const std::string &filename, bool realign = true);
    void render(meter_t antX, meter_t antY, degree_t antHeading) const;

    bool atDestination(meter_t x, meter_t y, meter_t threshold) const;
    std::tuple<meter_t, size_t> getDistanceToRoute(meter_t x, meter_t y) const;
    void setWaypointFamiliarity(size_t pos, double familiarity);
    void addPoint(meter_t x, meter_t y, bool error);

    size_t size() const{ return m_Waypoints.size(); }

    //------------------------------------------------------------------------
    // Operators
    //------------------------------------------------------------------------
    std::tuple<meter_t, meter_t, degree_t> operator[](size_t waypoint) const;

private:
    //------------------------------------------------------------------------
    // Members
    //------------------------------------------------------------------------
    GLuint m_WaypointsVAO;
    GLuint m_WaypointsPositionVBO;
    GLuint m_WaypointsColourVBO;

    GLuint m_RouteVAO;
    GLuint m_RoutePositionVBO;
    GLuint m_RouteColourVBO;
    unsigned int m_RouteNumPoints;

    std::vector<std::array<float, 2>> m_Waypoints;
    std::vector<degree_t> m_Headings;
    std::set<size_t> m_TrainedSnapshots;

    GLuint m_OverlayVAO;
    GLuint m_OverlayPositionVBO;
    GLuint m_OverlayColoursVBO;
};
}   // namespace AntWorld
}   // namespace BoBRobotics