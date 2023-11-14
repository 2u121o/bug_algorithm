#include "Point2d.hpp"

namespace typekit
{

Point2d::Point2d(const double x, const double y): x{x}, y{y}
{

}

std::ostream &operator<<(std::ostream &output, const Point2d &point2d)
{
    output << "x: " << point2d.x << std::endl <<  "y: " << point2d.y;
    return output;
}

}// typekit