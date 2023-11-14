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

void Bug::startWithTarget(const Eigen::Vector3d &target_state)
{
    target_state_ = target_state;

    std::cout << "[Bug] target state: " << target_state_ << std::endl;

}

void Bug::setTargetState(const Eigen::Vector3d &target_state)
{
    target_state_ = target_state;
}

}// mobile_robots
}// algorithm