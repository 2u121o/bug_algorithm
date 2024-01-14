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

bool Bug::isReoriented()
{
    const double EPSILON = 0.01;

    double orientation_desired = atan2(target_state_(0), target_state_(1))-M_PI/2;
    double orientation_current = robot_state_.theta;
    double orientation_error = orientation_desired - orientation_current;

    if(orientation_error>EPSILON || orientation_error<-EPSILON)
    {
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
    
    return true;
}

bool Bug::orientParallelToSurface(double &edge_angle)
{
    int NUM_POINTS = 10;    
    static int k=0;
    static std::vector<Eigen::Vector2d> points(10);
    Eigen::Vector2d point;

    static double x_mean = 0;
    static double y_mean = 0;
    if(ranges_(ranges_.size()/2-1)<=sensor_settings_.range_max && k < NUM_POINTS)
    {
        takeMeasurementsRange(original_map_, ranges_);
        point(0) = robot_state_.x + ranges_(ranges_.size()/2-1)*std::cos(robot_state_.theta);
        point(1) = robot_state_.y - ranges_(ranges_.size()/2-1)*std::sin(robot_state_.theta);
        x_mean += point(0);
        y_mean += point(1);
        points.at(k) =  point;

        moveRobot(original_map_,83);

        k++;
        std::cout << "Point: " << point << std::endl;
        // std::cout << "robot_state_.theta: " << robot_state_.theta << std::endl;

        return false;
    }
    x_mean /= k;
    y_mean /= k;

    double num = 0;
    double den = 0;
    for(short int j=0; j<k; ++j)
    {
         num += (points.at(j)(0)-x_mean)*(points.at(j)(1)-y_mean);
         den += std::pow(points.at(j)(0),2);
    }
    edge_angle = std::atan2(den, num);

    return true;
}

bool Bug::isMovingAroundObstacle()
{
    
    //find angle between the x of the world and the edge
    static double edge_angle;
    if(!is_oriented_parallel_to_surface_)
    {
        if(orientParallelToSurface(edge_angle))
        {
            is_oriented_parallel_to_surface_ = true;
        }
    }

    //orientation parallel to the obstacle edge
    const double EPSILON = 0.01;
    double orientation_current = robot_state_.theta;
    double orientation_error = -edge_angle - orientation_current;
    if(orientation_error>EPSILON || orientation_error<-EPSILON)
    {
        if(orientation_error>EPSILON)
        {
            moveRobot(original_map_,81);
        }
        else
        {
            moveRobot(original_map_,83);
        }
        orientation_current = robot_state_.theta;
        orientation_error = -edge_angle - orientation_current;
    }
    

    //TODO: move around the obstacle until it intercept the line that connect the starting point with 
    // the target
    return true;
}

bool Bug::nextStep()
{
    static bool is_moving_around_obstacle_started = false;
    if(!is_target_direction_)
    {
        is_target_direction_ = isReoriented();
        takeMeasurementsRange(original_map_, ranges_);
    }
    else
    {
        
        takeMeasurementsRange(original_map_, ranges_);
        if(ranges_(ranges_.size()/2-1) >= sensor_settings_.range_max && !is_moving_around_obstacle_started)
        {
            moveRobot(original_map_,82);
        }
        else
        {
            if(!isMovingAroundObstacle())
            {
                is_moving_around_obstacle_started = false;
                is_target_direction_ = false;
            }
            else
            {
                is_moving_around_obstacle_started = true;
            }
        }
        
    }
    return false;
    std::cout << "Reached!!!" << std::endl;
}

void Bug::startWithTarget(const cv::Mat &map, const Eigen::Vector3d &target_state)
{
    target_state_ = target_state;

    const double EPSILON = 0.01;

    double orientation_desired = atan2(target_state_(0), target_state_(1))-M_PI/2;
    // double orientation_desired = target_state(2);
    double orientation_current = robot_state_.theta;
    double orientation_error = orientation_desired - orientation_current;
    while(orientation_error>EPSILON || orientation_error<-EPSILON)
    {
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
    is_target_direction_ = false;
}

void Bug::setMap(const cv::Mat &original_map)
{
    original_map_ = original_map;
}

}// mobile_robots
}// algorithm