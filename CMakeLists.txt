cmake_minimum_required(VERSION 2.8.3)
project(cam_exploration)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf actionlib actionlib_msgs)
#find_package(Boost REQUIRED COMPONENTS pending/disjoint_sets)

add_action_files(
  DIRECTORY action
  FILES
  ExplorationServer.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

##add_library(cam_exploration_lib
##        src/frontier.cpp
##        src/frontiersMap.cpp
##        src/MapServer.cpp
##        src/MarkerPublisher.cpp
##        src/RobotMotion.cpp
##        src/strategy/AStar.cpp
##        src/strategy/frontierValue.cpp
##        src/strategy/goalSelector.cpp
##        src/strategy/replan.cpp
##)



add_executable(cam_exploration #src/cam_exploration.cpp
				src/exploration_server.cpp
				src/MarkerPublisher.cpp
				src/RobotMotion.cpp
				src/MapServer.cpp
				src/frontier.cpp
				src/frontiersMap.cpp
				src/strategy/frontierValue.cpp
				src/strategy/goalSelector.cpp
				src/strategy/replan.cpp
				src/strategy/AStar.cpp)



target_link_libraries(cam_exploration ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} )
##add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} tf)
