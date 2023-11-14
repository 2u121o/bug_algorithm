#ifndef POINT_2_D_HPP
#define POINT_2_D_HPP

#include <iostream>

namespace typekit
{

class Point2d
{

    public:

        double x;
        double y;
        
        Point2d() = default;
        Point2d(const double x, const double y);

        friend std::ostream& operator<<(std::ostream &output, const Point2d &point2d);

        
};


}// typekit

#endif