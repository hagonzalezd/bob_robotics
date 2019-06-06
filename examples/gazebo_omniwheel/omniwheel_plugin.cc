// BoB robotics includes
#include "common/circstat.h"

// Third-party includes
#include "third_party/units.h"

// Boost
#include <boost/bind.hpp>

// Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Standard C includes
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

// Standard C++ includes
#include <algorithm>
#include <chrono>
#include <iostream>
#include <vector>

using namespace BoBRobotics;
using namespace units::literals;
using namespace units::math;
using namespace units::angle;
using namespace units::frequency;
using namespace units::length;
using namespace units::angular_velocity;
using namespace units::velocity;
using namespace units::time;

constexpr meter_t halfWidth = 4_cm;
constexpr meter_t wheelRadius = 1.905_cm;
constexpr radian_t pirad{ pi() };

constexpr hertz_t P = 5_Hz;
constexpr hertz_t I = 0.0005_Hz;
constexpr meters_per_second_t Vmax = 1_mps;
constexpr radians_per_second_t omegamax = 50_rad_per_s;
constexpr meter_t stopDistance = 1_mm;
constexpr radian_t stopAngle = 12_deg;

meters_per_second_t Vx;
meters_per_second_t Vy;
meters_per_second_t Vxm;
meters_per_second_t Vym;
meters_per_second_t Vxw;
meters_per_second_t Vyw;
meters_per_second_t VxAbs;
meters_per_second_t VyAbs;
meters_per_second_t VxmTarget;
meters_per_second_t VymTarget;
meters_per_second_t VxwTarget;
meters_per_second_t VywTarget;
radians_per_second_t omegap;
meters_per_second_t Vback;
meters_per_second_t Vleft;
meters_per_second_t Vright;
meters_per_second_t VbackTarget;
meters_per_second_t VleftTarget;
meters_per_second_t VrightTarget;
radians_per_second_t omegapL;
radians_per_second_t omegapLVxm2;
meters_per_second_t sqrtVym2;
meters_per_second_t Vxm2;
meters_per_second_t Vback3;
meters_per_second_t Vleft3;
meters_per_second_t Vright3;
meters_per_second_t VxTarget;
meters_per_second_t VyTarget;
radians_per_second_t omegapTarget;
meter_t x = 0_m;
meter_t y = 0_m;
meter_t xm = 0_m;
meter_t ym = 0_m;
meter_t xOffset = 0_m;
meter_t yOffset = 0_m;
meter_t xTarget = 0_m;
meter_t yTarget = 0_m;
meter_t xw = 0_m;
meter_t yw = 0_m;
meter_t xError;
meter_t yError;
meter_t xErrorI;
meter_t yErrorI;
radian_t theta = 0_rad;
radian_t thetaError;
radian_t thetaErrorI;
radian_t thetaOffset = 0_rad;
radian_t thetaTarget = 0_rad;
second_t timeElapsed = 0_s;
second_t timeLast;
second_t timeNow = 0_s;
double backAngleElapsed;
double backAngleLast;
double backAngleNow = 0;
double leftAngleElapsed;
double leftAngleLast;
double leftAngleNow = 0;
double rightAngleElapsed;
double rightAngleLast;
double rightAngleNow = 0;
radian_t phi;
meters_per_second_t Vlow;
meters_per_second_t Vhigh;
double scale;

class Point
{
private:
    static meter_t w;

public:
    meter_t x, y;

    void offset(meter_t x, meter_t y)
    {
        this->x += x;
        this->y += y;
    }

    void rotate(radian_t theta)
    {
        w = (cos(theta) * this->y) + (sin(theta) * this->x);
        this->x = (cos(theta) * this->x) - (sin(theta) * this->y);
        this->y = w;
    }

    Point()
    {}

    Point(const Point &p)
    {
        x = p.x;
        y = p.y;
    }

    Point(meter_t x, meter_t y)
    {
        this->x = x;
        this->y = y;
    }

    Point(Point p, meter_t xOffset, meter_t yOffset)
    {
        x = p.x + xOffset;
        y = p.y + yOffset;
    }
};

std::vector<Point> points;
std::vector<radian_t> angles;

enum class Movement
{
    Direct,
    DirectM,
    DirectW,
    None,
    AbsoluteM,
    AbsoluteW,
    BezierW
};
Movement movement = Movement::None;

long long binomialCoefficient_c;
long long binomialCoefficient_i;

/**
 * source:
 * https://en.wikipedia.org/wiki/Binomial_coefficient
 * @param n the number that goes above`
 * @param k the number that goes below
 * @return
 */
long long
binomialCoefficient(long long n, long long k)
{
    if ((k < 0) || (k > n))
        return 0;
    if ((k == 0) || (k == n))
        return 1;
    k = std::min(k, n - k); // take advantage of symmetry
    binomialCoefficient_c = 1;
    for (long binomialCoefficient_i = 0;
         binomialCoefficient_i < k;
         binomialCoefficient_i++)
        binomialCoefficient_c = binomialCoefficient_c * (n - binomialCoefficient_i) / (binomialCoefficient_i + 1);
    return binomialCoefficient_c;
}

double Bezier_t;
double Bezier_step;
long long Bezier_nT;
long long Bezier_nR;
long long Bezier_i;
double Bezier_B;
meter_t Bezier_pX, Bezier_pY;
radian_t Bezier_pTheta;

double
Bezier_B_(double Bezier_n)
{
    return ((double) binomialCoefficient(Bezier_n, Bezier_i))
            * std::pow((double) Bezier_t, (double) Bezier_i)
            * std::pow(1.0 - Bezier_t, (double) (Bezier_n - Bezier_i));
}

void
Bezier()
{
    Bezier_pX = 0_m;
    Bezier_pY = 0_m;
    for (Bezier_i = 0; Bezier_i <= Bezier_nT; Bezier_i++) {
        Bezier_B = Bezier_B_(Bezier_nT);
        Bezier_pX += points[Bezier_i].x * Bezier_B;
        Bezier_pY += points[Bezier_i].y * Bezier_B;
    }
    Bezier_pTheta = 0_rad;
    for (Bezier_i = 0; Bezier_i <= Bezier_nR; Bezier_i++) {
        Bezier_pTheta += angles[Bezier_i] * Bezier_B_(Bezier_nR);
    }
}

/**
 * Resets the error values used by the PID controller.
 *
 * Should be used after a movement command if another one is fired
 * before or immediately after the previous one
 * and a continuous movement is not desired.
 */
void
resetErrors()
{
    xErrorI = 0_m;
    yErrorI = 0_m;
    thetaErrorI = 0_rad;
}

/**
 * Calculates the mobile platform's velocities
 * relative to the mobile platform's frame
 * from the wheels velocities.
 */
void
forwardKinematicsMobile()
{
    Vleft3 = Vleft / 3.0;
    Vback3 = Vback / 3.0;
    Vright3 = Vright / 3.0;
    Vxm = (2 * Vback3) - Vleft3 - Vright3;
    Vym = sqrt(3.0) * (Vright3 - Vleft3);
    omegap = getAngularVelocity(Vleft3 + Vback3 + Vright3, halfWidth);
}

/**
 * Calculates the mobile platform's velocities
 * relative to the world's frame
 * from the wheels velocities.
 *
 * Also runs forwardKinematicsMobile().
 */
void
forwardKinematicsWorld()
{
    forwardKinematicsMobile();
    Vxw = (cos(theta) * Vxm) - (sin(theta) * Vym);
    Vyw = (cos(theta) * Vym) + (sin(theta) * Vxm);
}

/**
 * Generates a pseudo random number between 0 and maximumValue, inclusive.
 * @param maximumValue
 * @return
 */
double
getRandom(double maximumValue)
{
    return (((double) std::rand()) / ((double) RAND_MAX)) * maximumValue;
}

/**
 * Calculates the wheels velocities
 * from the mobile platform's velocities
 * relative to the mobile platform's frame.
 */
void
inverseKinematicsMobile()
{
    omegapL = omegap * halfWidth.value();
    sqrtVym2 = Vym * sqrt(3.0) / 2.0;
    Vxm2 = Vxm / 2.0;
    omegapLVxm2 = radians_per_second_t{ omegapL() - Vxm2() };
    Vleft = meters_per_second_t{ omegapLVxm2() - sqrtVym2() };
    Vback = meters_per_second_t{ omegapL() + Vxm() };
    Vright = meters_per_second_t{ omegapLVxm2() + sqrtVym2() };
}

/**
 * Calculates the wheels velocities
 * from the mobile platform's velocities
 * relative to the mobile platform's frame.
 *
 * Also runs inverseKinematicsMobile().
 */
void
inverseKinematicsWorld()
{
    Vxm = (cos(theta) * Vxw) + (sin(theta) * Vyw);
    Vym = (cos(theta) * Vyw) - (sin(theta) * Vxw);
    inverseKinematicsMobile();
}

bool
isStopPose()
{
    return (abs(xError) < stopDistance) && (abs(yError) < stopDistance) && (abs(thetaError) < stopAngle);
}

namespace gazebo {

class OmniPlatformPlugin : public ModelPlugin
{
public:
    /**
     * Movement for desired pose in mobile platform's frame.
     *
     * Translates the desired pose in the mobile platform's frame
     * to the world's frame and moves according to the world's frame.
     * Keep in mind that how the mobile platform reaches the target theta
     * will affect the platform's position in it's own frame,
     * and that position can't be predicted. Also remember
     * that you can change the value of the platform's pose in either frame.
     * To be executed by the communications module.
     *
     * @param x
     * @param y
     * @param theta
     */
    void fireMovementAbsoluteM(meter_t x, meter_t y, radian_t theta)
    {
        // distance to be traversed in the mobile platform's frame
        xTarget = x - xm;
        yTarget = y - ym;
        // distance rotated to the direction
        // to be traversed in the world's frame
        x = (cos(theta) * xTarget) - (sin(theta) * yTarget);
        y = (cos(theta) * yTarget) + (sin(theta) * xTarget);
        // rotated distance added to current world's frame position
        xTarget = xw + x;
        yTarget = yw + y;
        thetaTarget = theta;
        movement = Movement::AbsoluteW;
        updateIndicator();
    }

    /**
     * Random movement for desired pose in mobile platform's frame
     *
     * At the end of the movement, the mobile platform's x, y and theta
     * will be the same as the parameters. However, the position
     * in the world frame will depend on how theta will vary
     * from it's current value to the target value, and can't be predicted.
     * It's preferred to use fireMovementPoseM(),
     * unless you know what you're doing.
     * To be executed by the communications module.
     *
     * @param x
     * @param y
     * @param theta
     */
    void fireMovementAbsoluteMRaw(meter_t x, meter_t y, radian_t theta)
    {
        xTarget = x;
        yTarget = y;
        thetaTarget = theta;
        movement = Movement::AbsoluteM;
        updateIndicator();
    }

    /**
     * Movement for desired pose in world frame.
     *
     * To be executed by the communications module.
     *
     * @param x
     * @param y
     * @param theta
     */
    void fireMovementAbsoluteW(meter_t x, meter_t y, radian_t theta)
    {
        xTarget = x;
        yTarget = y;
        thetaTarget = theta;
        movement = Movement::AbsoluteW;
        updateIndicator();
    }

    /**
     * Incremental movement based on two Bézier curves
     * relative to the platform's frame.
     *
     * The curve may be translated such that the first control point
     * and angle coincides with the current platform's position and angle.
     *
     * The Bézier function's <code>t</code> variable, by default, iterates
     * between 0 and 1 to generate the curve. <code>t</code>'s step value
     * is defined by the user, and influences the time the platform
     * will take to execute the movement, which is equal to the step value
     * divided by the iteration time. If the movement time is too short
     * (which depends on the movement length and complexity), the platform
     * will make it's best effort to execute the movement. Therefore,
     * the longer the movement time (the shorter the step value), the closer
     * the platform will be to the desired movement.
     *
     * The platform's angle is also controlled by a Bézier curve. However,
     * the algorithm has been modified to be one dimensional, which means
     * its input are control values instead of control points.
     * If a constant angle is desired, input a vector with a single value
     * equal to the angle.
     *
     * @param step Bézier funcion's <code>t</code> variable's increment value.
     * @param points control points for the translation movement
     * @param angles control angles for the rotation movement
     * @param offsetT whether to offset the control points to the starting platform's position
     * @param offsetR whether to offset the control angles to the starting platform's angle
     */
    void fireMovementBezierM(std::vector<Point> *points,
                             std::vector<radian_t> *angles,
                             double step,
                             bool offsetT,
                             bool offsetR)
    {
        if (points->empty() || angles->empty())
            return;
        ::points.clear();
        ::angles.clear();
        Bezier_nT = points->size() - 1;
        Bezier_nR = angles->size() - 1;
        for (Bezier_i = 0; Bezier_i <= Bezier_nT; Bezier_i++) {
            ::points.push_back(Point(points->at(Bezier_i)));
            ::points[Bezier_i].offset(-xw, -yw);
            ::points[Bezier_i].rotate(theta);
            ::points[Bezier_i].offset(xw, yw);
        }
        if (offsetT) {
            xOffset = xw - ::points[0].x;
            yOffset = yw - ::points[0].y;
        } else {
            xOffset = 0_m;
            yOffset = 0_m;
        }
        for (Bezier_i = 0; Bezier_i <= Bezier_nT; Bezier_i++) {
            ::points[Bezier_i].offset(xOffset, yOffset);
        }
        if (offsetR) {
            thetaOffset = pirad - normaliseAngle360((*angles)[0] + pirad - theta);
        } else {
            thetaOffset = 0_rad;
        }
        for (Bezier_i = 0; Bezier_i <= Bezier_nR; Bezier_i++) {
            ::angles.push_back(angles->at(Bezier_i) + thetaOffset);
        }
        Bezier_t = 0;
        Bezier_step = step;
        movement = Movement::BezierW;
        updateIndicator();
    }

    /**
     * Incremental movement based on two Bézier curves relative to the world's frame.
     *
     * The curve may be translated such that the first control point
     * and angle coincides with the current platform's position and angle.
     *
     * The Bézier function's <code>t</code> variable, by default, iterates
     * between 0 and 1 to generate the curve. <code>t</code>'s step value
     * is defined by the user, and influences the time the platform
     * will take to execute the movement, which is equal to the step value
     * divided by the iteration time. If the movement time is too short
     * (which depends on the movement length and complexity), the platform
     * will make it's best effort to execute the movement. Therefore,
     * the longer the movement time (the shorter the step value), the closer
     * the platform will be to the desired movement.
     *
     * The platform's angle is also controlled by a Bézier curve. However,
     * the algorithm has been modified to be one dimensional, which means
     * its input are control values instead of control points.
     * If a constant angle is desired, input a vector with a single value
     * equal to the angle.
     *
     * @param step Bézier funcion's <code>t</code> variable's increment value.
     * @param points control points for the translation movement
     * @param angles control angles for the rotation movement
     * @param offsetT whether to offset the control points to the starting platform's position
     * @param offsetR whether to offset the control angles to the starting platform's angle
     */
    void fireMovementBezierW(std::vector<Point> *points,
                             std::vector<radian_t> *angles,
                             double step,
                             bool offsetT,
                             bool offsetR)
    {
        if (points->empty() || angles->empty())
            return;
        ::points.clear();
        ::angles.clear();
        Bezier_nT = points->size() - 1;
        Bezier_nR = angles->size() - 1;
        if (offsetT) {
            xOffset = xw - (*points)[0].x;
            yOffset = yw - (*points)[0].y;
        } else {
            xOffset = 0_m;
            yOffset = 0_m;
        }
        if (offsetR) {
            thetaOffset = pirad - normaliseAngle360((*angles)[0] + pirad - theta);
        } else {
            thetaOffset = 0_rad;
        }
        for (Bezier_i = 0; Bezier_i <= Bezier_nT; Bezier_i++) {
            ::points.push_back(Point(points->at(Bezier_i), xOffset, yOffset));
        }
        for (Bezier_i = 0; Bezier_i <= Bezier_nR; Bezier_i++) {
            ::angles.push_back(angles->at(Bezier_i) + thetaOffset);
        }
        Bezier_t = 0;
        Bezier_step = step;
        movement = Movement::BezierW;
        updateIndicator();
    }

    /**
     * Moves the platform with fixed translation and rotation speeds
     * relative to the mobile platform's frame but translated
     * to the world's frame. This results in a world's frame movement
     * rotated according to the initial angle between frames.
     *
     * To be executed by the communications module.
     *
     * @param Vleft
     * @param Vback
     * @param Vright
     */
    void fireMovementDirectHybrid(meters_per_second_t Vxm, meters_per_second_t Vym, radians_per_second_t omegap)
    {
        VxTarget = (cos(theta) * Vxm) - (sin(theta) * Vym);
        VyTarget = (cos(theta) * Vym) + (sin(theta) * Vxm);
        omegapTarget = omegap;
        movement = Movement::DirectW;
    }

    /**
     * Moves the platform with fixed translation and rotation speeds,
     * relative to the mobile platform's frame. Does not transform
     * the speeds from the mobile platform's frame to the world frame.
     * This means that random changes in the platform's angle
     * will not change it's movement, as the mobile platform's frame
     * rotates with the platform.
     *
     * To be executed by the communications module.
     *
     * @param Vleft
     * @param Vback
     * @param Vright
     */
    void fireMovementDirectMobile(meters_per_second_t Vxm, meters_per_second_t Vym, radians_per_second_t omegap)
    {
        VxTarget = Vxm;
        VyTarget = Vym;
        omegapTarget = omegap;
        movement = Movement::DirectM;
    }

    /**
     * Sets the wheels' speeds, in m/s.
     *
     * To be executed by the communications module.
     * @param Vleft
     * @param Vback
     * @param Vright
     */
    void fireMovementDirectWheel(meters_per_second_t Vleft, meters_per_second_t Vback, meters_per_second_t Vright)
    {
        VleftTarget = Vleft;
        VbackTarget = Vback;
        VrightTarget = Vright;
        movement = Movement::Direct;
    }

    /**
     * Moves the platform with fixed translation and rotation speeds,
     * relative to the world's frame.
     *
     * To be executed by the communications module.
     *
     * @param Vleft
     * @param Vback
     * @param Vright
     */
    void fireMovementDirectWorld(meters_per_second_t Vxw, meters_per_second_t Vyw, radians_per_second_t omegap)
    {
        VxTarget = Vxw;
        VyTarget = Vyw;
        omegapTarget = omegap;
        movement = Movement::DirectW;
    }

    /**
     * Movement of a given distance and angle of rotation
     * in the mobile platform's frame
     *
     * Translates the movement in the mobile platform's frame
     * to the world's frame and moves according to the world's frame.
     * Keep in mind that how the mobile platform reaches the target theta
     * will affect the platform's position in it's own frame,
     * and that position can't be predicted. Also remember
     * that you can change the value of the platform's pose in either frame.
     * To be executed by the communications module.
     *
     * @param x
     * @param y
     * @param theta
     */
    void fireMovementRelativeM(meter_t x, meter_t y, radian_t theta)
    {
        // distance to be traversed in the mobile platform's frame
        xTarget = x;
        yTarget = y;
        // distance rotated to the direction
        // to be traversed in the world's frame
        x = (cos(theta) * xTarget) - (sin(theta) * yTarget);
        y = (cos(theta) * yTarget) + (sin(theta) * xTarget);
        // rotated distance added to current world's frame position
        xTarget = xw + x;
        yTarget = yw + y;
        thetaTarget = normaliseAngle360(::theta + theta);
        movement = Movement::AbsoluteW;
        updateIndicator();
    }

    /**
     * Random movement of a given distance and angle of rotation
     * in the mobile platform's frame
     *
     * At the end of the movement, the mobile platform will have traversed
     * in x and y and rotated by theta, relative to its frame.
     * However, the position in the world frame will depend
     * on how theta will vary from it's current value to the target value,
     * and can't be predicted. It's preferred to use fireMovementPoseM(),
     * unless you know what you're doing.
     * To be executed by the communications module.
     *
     * @param x
     * @param y
     * @param theta
     */
    void fireMovementRelativeMRaw(meter_t x, meter_t y, radian_t theta)
    {
        xTarget = xm + x;
        yTarget = ym + y;
        thetaTarget = normaliseAngle360(::theta + theta);
        movement = Movement::AbsoluteM;
        updateIndicator();
    }

    /**
     * Movement of a given distance and angle of rotation
     * relative to the world frame.
     *
     * To be executed by the communications module.
     *
     * @param x
     * @param y
     * @param theta
     */
    void fireMovementRelativeW(meter_t x, meter_t y, radian_t theta)
    {
        xTarget = xw + x;
        yTarget = yw + y;
        thetaTarget = normaliseAngle360(::theta + theta);
        movement = Movement::AbsoluteW;
        updateIndicator();
    }

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        this->model = _parent;

        this->leftJoint = model->GetJoint("left_joint");
        this->backJoint = model->GetJoint("back_joint");
        this->rightJoint = model->GetJoint("right_joint");

        this->world = _parent->GetWorld();
        //this->indicator = this->world->GetModel("wood_cube_2_5cm");
        //this->indicatorPose.reset(new math::Pose());
        //this->indicator->SetGravityMode(false);
        //this->indicator->Fini();

        std::srand(std::time(0));

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                boost::bind(&OmniPlatformPlugin::OnUpdate, this, _1));
    }

    // Called by the world update start event
    void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
        odometry();
        switch (movement) {
        case Movement::AbsoluteM:
            xError = xTarget - xm;
            yError = yTarget - ym;
            // method parameter: theta plus shift to place thetaTarget at pi
            thetaError = pirad - normaliseAngle360(theta + pirad - thetaTarget);
            if (isStopPose()) {
                movement = Movement::None;
                break;
            }
            xErrorI += xError;
            yErrorI += yError;
            thetaErrorI += thetaError;
            Vxm = (xError * P) + (xErrorI * I);
            Vym = (yError * P) + (yErrorI * I);
            VxAbs = abs(Vxm);
            VyAbs = abs(Vym);
            if ((VxAbs > Vmax) || (VyAbs > Vmax)) {
                scale = Vmax / ((VxAbs > VyAbs) ? VxAbs : VyAbs);
                Vxm *= scale;
                Vym *= scale;
            }
            omegap = (thetaError * P) + (thetaErrorI * I);
            if (omegap > omegamax) {
                omegap = omegamax;
            }
            inverseKinematicsMobile();
            break;
        case Movement::BezierW:
            if (Bezier_t <= 1) {
                Bezier();
                xTarget = Bezier_pX;
                yTarget = Bezier_pY;
                thetaTarget = normaliseAngle360(Bezier_pTheta);
                Bezier_t += Bezier_step;
                updateIndicator();
            } else {
                movement = Movement::None;
            }
            // falls through
        case Movement::AbsoluteW:
            xError = xTarget - xw;
            yError = yTarget - yw;
            // method parameter: theta plus shift to place thetaTarget at pi
            thetaError = pirad - normaliseAngle360(theta + pirad - thetaTarget);
            if (isStopPose() && (movement == Movement::AbsoluteW)) {
                movement = Movement::None;
                break;
            }
            xErrorI += xError;
            yErrorI += yError;
            thetaErrorI += thetaError;
            Vxw = (xError * P) + (xErrorI * I);
            Vyw = (yError * P) + (yErrorI * I);
            VxAbs = abs(Vxw);
            VyAbs = abs(Vyw);
            if ((VxAbs > Vmax) || (VyAbs > Vmax)) {
                scale = Vmax / ((VxAbs > VyAbs) ? VxAbs : VyAbs);
                Vxw *= scale;
                Vyw *= scale;
            }
            omegap = (thetaError * P) + (thetaErrorI * I);
            if (omegap > omegamax) {
                omegap = omegamax;
            }
            inverseKinematicsWorld();
            break;
        case Movement::Direct:
            Vleft = VleftTarget;
            Vback = VbackTarget;
            Vright = VrightTarget;
            break;
        case Movement::DirectM:
            Vxm = VxTarget;
            Vym = VyTarget;
            omegap = omegapTarget;
            inverseKinematicsMobile();
            break;
        case Movement::DirectW:
            Vxw = VxTarget;
            Vyw = VyTarget;
            omegap = omegapTarget;
            inverseKinematicsWorld();
            break;
        default:
            xErrorI = 0_m;
            yErrorI = 0_m;
            thetaErrorI = 0_rad;
            Vleft = 0_mps;
            Vback = 0_mps;
            Vright = 0_mps;
            //*
            fireMovementAbsoluteW(meter_t{ getRandom(1) - 0.5 },
                                  meter_t{ getRandom(1) - 0.5 },
                                  radian_t{ getRandom(2 * pi()) });
            /**/
        }

        leftJoint->SetVelocity(0, getAngularVelocity(Vleft, wheelRadius).value());
        backJoint->SetVelocity(0, getAngularVelocity(Vback, wheelRadius).value());
        rightJoint->SetVelocity(0, getAngularVelocity(Vright, wheelRadius).value());
    }

private:
    physics::JointPtr backJoint;
    physics::JointPtr leftJoint;
    physics::JointPtr rightJoint;

    // Pointer to the model
    physics::ModelPtr model;

    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;

    physics::WorldPtr world;

    void odometry()
    {
        timeLast = timeNow;
        timeNow = second_t{ this->world->SimTime().Double() };
        timeElapsed = timeNow - timeLast;
        Vleft = meters_per_second_t{ this->leftJoint->GetVelocity(0) * wheelRadius.value() };
        Vback = meters_per_second_t{ this->backJoint->GetVelocity(0) * wheelRadius.value() };
        Vright = meters_per_second_t{ this->rightJoint->GetVelocity(0) * wheelRadius.value() };
        theta = normaliseAngle360(radian_t{ model->RelativePose().Rot().Yaw() }); // simulation theta
        forwardKinematicsWorld();
        xm += Vxm * timeElapsed;
        ym += Vym * timeElapsed;
        xw += Vxw * timeElapsed;
        //xw = model->GetRelativePose().pos.x; // simulation x
        yw += Vyw * timeElapsed;
        //yw = model->GetRelativePose().pos.y; // simulation y
    }

    /**
     * Simulation purposes only.
     */
    void updateIndicator()
    {
        //            indicatorPose->Set(xTarget, yTarget, 0.05, 0, 0, thetaTarget);
        //            indicator->SetRelativePose(*indicatorPose);
    }
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(OmniPlatformPlugin)
}

// C++ bug: when things are declared as 'static' or 'const',
// they lose their references, and it's only possible to read from them
// to restore their references, they must be decladed again like this:
meter_t Point::w;
