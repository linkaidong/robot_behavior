cmake_minimum_required(VERSION 3.0.2)
project(robot_decision)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED)

set(ROS_DEPENDENCIES
        roscpp
        behaviortree_cpp_v3
        geometry_msgs
        std_msgs
        tf2
        move_base_msgs
        actionlib
        rm_common
        )

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES})

catkin_package(
         INCLUDE_DIRS
         include
         LIBRARIES
         CATKIN_DEPENDS
         ${ROS_DEPENDENCIES}
)

include_directories(
         include
        ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}  src/main.cpp
                                src/patrol.cpp
                                src/is_enemy_visible.cpp
                                src/track_to_attack.cpp
                                src/choosemode.cpp
                                src/navigation.cpp
                                src/manual.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


