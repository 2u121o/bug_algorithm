#include <iostream>

#include "Bug.hpp"

int main()
{

    Eigen::Vector3d initial_state;
    initial_state << 100, 100, 0;

    Eigen::Vector3d target_state;
    target_state << 500, 500, 0;

    std::string paht_map = "../../maps/map_obstacles.png";
    cv::Mat map = cv::imread(paht_map);

    const int RADIUS = 5;
    const bool ADD_NOISE = false;
    algorithm::mobile_robots::Bug bug(map, initial_state, RADIUS, ADD_NOISE);

    bug.drawRobot(map);
    bug.startWithTarget(target_state);

    Eigen::VectorXd ranges; 
    bug.takeMeasurementsRange(map, ranges);

    cv::imshow("Map", map);
    int k = cv::waitKey(0);

    return 0;
}