#ifndef BUG_HPP
#define BUG_HPP

#include <opencv2/opencv.hpp>

#include <mobile_robot_model/Robot.hpp>

/**
 * @brief The Bug object extend the Robot object which represent a circular mobile
 *        platform provided with 2d laser scanner. The Bug in addition make this 
 *        platform autonomous. In the sense that given a starting point and a desired
 *        location, it is able to find the path avoiding the possible obstacles.
 * 
 */

namespace algorithm
{
namespace mobile_robots
{

class Bug : public Robot
{

    public:
        Bug() = default;
        using Robot::Robot;

        void start();
        void startWithTarget(const cv::Mat &map, const Eigen::Vector3d &target_state);

        void setTargetState(const Eigen::Vector3d &target_state);

        bool nextStep();

        void setMap(const cv::Mat &original_map);

    private:
        Eigen::Vector3d target_state_;

        cv::Mat original_map_;
};

}// mobile_robots
}// algorithm

#endif
