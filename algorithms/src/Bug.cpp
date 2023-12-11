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

bool Bug::nextStep()
{
    const double EPSILON = 0.01;

    double orientation_desired = atan2(target_state_(0), target_state_(1))-M_PI/2;
    // double orientation_desired = target_state(2);
    double orientation_current = robot_state_.theta;
    double orientation_error = orientation_desired - orientation_current;
    std::cout << "orientation_desired: " << orientation_desired << std::endl; 
    if(orientation_error>EPSILON || orientation_error<-EPSILON)
    {
        std::cout << "[bug] adjusting orientation: " << orientation_current << std::endl;
        //std::cout << "orientation_error: " << orientation_error << std::endl; 
        if(orientation_error>EPSILON)
        {
            moveRobot(original_map_,81);
        }
        else
        {
            moveRobot(original_map_,83);
        }
        orientation_current = robot_state_.theta;
        orientation_error = orientation_desired - orientation_current;
        return false;
    }
    std::cout << "Reached!!!" << std::endl;
    return true;
}

void Bug::startWithTarget(const cv::Mat &map, const Eigen::Vector3d &target_state)
{
    target_state_ = target_state;

    // cv::Point target_point(target_state_(0), target_state_(1));
    // cv::circle(map, target_point, 1, cv::Scalar(0, 255, 0), 5, 8, 0);

    std::cout << "[Bug] target state: " << target_state_ << std::endl;

    const double EPSILON = 0.01;

    double orientation_desired = atan2(target_state_(0), target_state_(1))-M_PI/2;
    // double orientation_desired = target_state(2);
    double orientation_current = robot_state_.theta;
    double orientation_error = orientation_desired - orientation_current;
    std::cout << "orientation_desired: " << orientation_desired << std::endl; 
    while(orientation_error>EPSILON || orientation_error<-EPSILON)
    {
        std::cout << "[bug] adjusting orientation: " << orientation_current << std::endl;
        //std::cout << "orientation_error: " << orientation_error << std::endl; 
        if(orientation_error>EPSILON)
        {
            moveRobot(map,81);
        }
        else
        {
            moveRobot(map,83);
        }
        orientation_current = robot_state_.theta;
        orientation_error = orientation_desired - orientation_current;
    }
    std::cout << "Reached!!!" << std::endl;

}

void Bug::setTargetState(const Eigen::Vector3d &target_state)
{
    target_state_ = target_state;
}

void Bug::setMap(const cv::Mat &original_map)
{
    original_map_ = original_map;
}

}// mobile_robots
}// algorithm