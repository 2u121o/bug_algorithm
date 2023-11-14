#ifndef POSE_2_D_HPP
#define POSE_2_D_HPP

#include <iostream>

#include "Point2d.hpp"

namespace typekit
{

class Pose2d
{
    public:

        Point2d position;
        double orientation;

        Pose2d() = default;
        Pose2d(const Point2d &position, const double orientation);
};


}// typekit

#endif