#include "Bug.hpp"

namespace algorithm
{
namespace mobile_robots
{

void Bug::start()
{
    //TODO: implement here the bug 1 algorithm to go from one point to another
    //      in presence of obstacles.


}

void Bug::startWithTarget(cv::Mat &map, const Eigen::Vector3d &target_state)
{
    target_state_ = target_state;
    original_map_ = map.clone();

    cv::Point target_point(target_state_(0), target_state_(1));
    cv::circle(map, target_point, 1, cv::Scalar(0, 255, 0), 5, 8, 0);

    std::cout << "[Bug] target state: " << target_state_ << std::endl;

    const double EPSILON = 0.01;

    double orientation_desired = atan2(target_state_(0), target_state_(1))-M_PI/2;
    double orientation_current = state_(2);
    double orientation_error = orientation_desired - orientation_current;
    std::cout << "orientation_desired: " << orientation_desired << std::endl; 
    while(orientation_error>EPSILON || orientation_error<-EPSILON)
    {
        original_map_ = map.clone();
        //std::cout << "orientation_error: " << orientation_error << std::endl; 
        if(orientation_error>EPSILON)
        {
            moveRobot(original_map_,81);
        }
        else
        {
            moveRobot(original_map_,83);
        }
        orientation_current = state_(2);
        orientation_error = orientation_desired - orientation_current;
        drawRobot(original_map_);
    }
    
    std::cout << "Reached!!!" << std::endl;

}

void Bug::setTargetState(const Eigen::Vector3d &target_state)
{
    target_state_ = target_state;
}

}// mobile_robots
}// algorithm