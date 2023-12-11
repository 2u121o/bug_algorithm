#include <iostream>

#include <unistd.h>

#include <mobile_robot_model/World.hpp>
#include <mobile_robot_model/typekit.hpp>

#include "Bug.hpp"

int main()
{

    RobotState initial_state;
    initial_state.x = 100.0;
    initial_state.y = 100.0;
    initial_state.theta =  M_PI/2;

    Eigen::Vector3d target_state;
    target_state << 500, 500, 0;

    std::string paht_map = "../../maps/map_obstacles.png";
    cv::Mat original_map = cv::imread(paht_map);
    cv::Mat map = cv::imread(paht_map);

    const int RADIUS = 15;
    const bool ADD_NOISE = false;
    algorithm::mobile_robots::Bug bug(original_map, initial_state, RADIUS, ADD_NOISE);

    World world;
    int visualization_time = 25;
    world.setMap(map);
    world.setRobot(bug);
    world.drawWorld(visualization_time);

    bug.setTargetState(target_state);
    bug.setMap(original_map);

    bool target_reached = false;
    while(!target_reached)
    {
        target_reached = bug.nextStep();
        map = cv::imread(paht_map);
        cv::Point target_point(target_state(0), target_state(1));
        cv::circle(map, target_point, 1, cv::Scalar(0, 255, 0), 5, 8, 0);
        world.setMap(map);
        world.setRobot(bug);
        world.drawWorld(visualization_time);

    }

    visualization_time = 0;
    cv::Point target_point(target_state(0), target_state(1));
    cv::circle(map, target_point, 1, cv::Scalar(0, 255, 0), 5, 8, 0);
    world.drawWorld(visualization_time);
    
    Eigen::VectorXd ranges; 
    //bug.takeMeasurementsRange(map, ranges);

    // cv::imshow("Map", original_map);
    // int k = cv::waitKey(0);

    return 0;
}