cmake_minimum_required(VERSION 3.10)

project(bug_one)

include_directories(include)

set(SOURCE src/Bug.cpp src/main.cpp)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(mobile_robot_model REQUIRED)
include_directories(${mobile_robot_model_INCLUDE_DIRS})

find_package(OpenCV  REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${SOURCE})
target_link_libraries(${PROJECT_NAME}
                                        ${Eigen3} 
                                        ${mobile_robot_model_LIBRARIES}
                                        ${OpenCV_LIBS}
                      )


