/*
 * exploration_server.cpp
 *
 *  Created on: Dec 4, 2017
 *      Author: vik
 */




#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <cam_exploration/ExplorationServerAction.h>
#include <cam_exploration/MapServer.h>
#include <cam_exploration/goalSelector.h>
#include <cam_exploration/replan.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>

using namespace std;
using namespace cam_exploration::strategy;

/**
 * @file cam_exploration.cpp
 * @brief Top level file for exploration with an RGBA camera.
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-19
 */


/**
 * @brief Alias for integer iterator loops
 */
typedef vector<int>::const_iterator viit;

/**
 * @brief Typedef for the simple action server
 */
typedef actionlib::SimpleActionServer<cam_exploration::ExplorationServerAction> ActionServer;

/**
 * @brief Top level namespace of the project cam_exploration
 */
namespace cam_exploration{


//! Robot motion handler
RobotMotion robot;
//! List of all frontiers in the occupancy grid
FrontiersMap fmap;
//! Object that decides the actual robot goal given a certain frontier
strategy::goalSelector* goal_selector;


/**
 * @brief Choose a proper goal pose to sent to move_base
 *
 * @return Goal pose
 */
geometry_msgs::Pose decideGoal()
{
    geometry_msgs::Pose goal;
    FrontiersMap::const_iterator f_it = fmap.begin();
    while(!goal_selector->decideGoal(*f_it, goal)){
        f_it++;
    }

    MarkerPublisher markers;
    markers.publish("f_goal", f_it->points);
    markers.publish("goal", goal);
    return goal;
}



/**
 * @brief Callback to update the frontiers map according to new MapServer information
 *
 * @param f_in MapServer's frontier map
 */
void getFrontiers(FrontiersMap& f_in)
{
    fmap = f_in;
}

float getDistance2D(geometry_msgs::Point init_p, geometry_msgs::Point final_p)
{
  return sqrt(pow(final_p.x - init_p.x, 2) + pow(final_p.y - init_p.y, 2));
}

/**
 * @brief Node's main loop
 */
void ExecuteAction(const cam_exploration::ExplorationServerGoalConstPtr goal, ActionServer* as_)
{
    if(goal->ExplorationGoal){
      as_->acceptNewGoal();
    } else {
      ROS_WARN("Goal not accepted.");
      return;
    }


    // >> Exploration objects
    MapServer mapServer;
    replan::Replaner replaner;


    ros::spinOnce();
    //ros::NodeHandle n;
    ros::NodeHandle nh("~");


    // >> Subscribing map server to Map
    mapServer.subscribeMap("proj_map", getFrontiers, ros::NodeHandlePtr(new ros::NodeHandle()),
                                ros::NodeHandlePtr(new ros::NodeHandle("~")));
    // >> Subscribing costmap server to Map
    mapServer.subscribeCostMap("/move_base/global_costmap/costmap", ros::NodeHandlePtr(new ros::NodeHandle()),
                                ros::NodeHandlePtr(new ros::NodeHandle("~")));

    // >> Setting up replaning function
    vector<string> replan_conditions;
    ros::NodeHandle n_replan(nh, "replaning");
    n_replan.getParam("conditions", replan_conditions);


    for(vector<string>::iterator it = replan_conditions.begin(); it != replan_conditions.end(); ++it){
        map<string, string> parameters;

        ROS_INFO("processing %s", it->c_str());
        if (n_replan.getParam(it->c_str(), parameters)){
            replaner.addCause(it->c_str(), parameters);
        }
        else{
            ROS_INFO("No parameters found for replaning cause %s", it->c_str());
            replaner.addCause(it->c_str());
        }
    }


    // >> decideGoal setup
    string g_selector_name;

    if( nh.getParam("goal_selector/type", g_selector_name)){
        if (g_selector_name == "mid_point"){
            goal_selector = new midPoint();
        }
        else{
            ROS_ERROR("String %s does not name a valid goal selector", g_selector_name.c_str());
        }
    }
    else
        ROS_ERROR("Parameter goal_selector has not been configured");



    // MarkerPublisher setup
    MarkerPublisher markers;
    markers.add("f_goal", "goal_frontier");
    markers.add("goal", "goal_marker");

    // Arrow shape
    geometry_msgs::Vector3 scale;
    scale.x = 0.5;
    scale.y = 0.2;
    scale.z = 0.1;

    int type = visualization_msgs::Marker::ARROW;
    markers.setProperty("goal", scale);
    markers.setProperty("goal", type);

    bool first_time = true;

    if(mapServer.mapReceived())
    {
      mapServer.setMapReceived();
      ROS_INFO_ONCE("First map received!");

          if(robot.refreshPose())
          {
              if(replaner.replan() || first_time)
              {
                  if (first_time)
                      first_time = false;

                  robot.printStatus();
                  robot.goTo(decideGoal());

                  geometry_msgs::Point current_goal = robot.current_frontier_target_point();
                  geometry_msgs::Point initial_robot_position = robot.position();
                  ros::Time initial_time = ros::Time::now();
                  ros::Duration elapsed_time;
                  float distance_covered = 0;
                  float distance_to_goal = 0;

                  cam_exploration::ExplorationServerResult result;

                  ros::Rate loop_rate(1);
                  while(robot.status() == robot.MOVING)
                  {
                    if(robot.refreshPose())
                    {
                      elapsed_time = ros::Time::now() - initial_time;
                      distance_covered = cam_exploration::getDistance2D(initial_robot_position, robot.position());
                      distance_to_goal = cam_exploration::getDistance2D(robot.position(), current_goal);
                      cam_exploration::ExplorationServerFeedback feedback_;
                      feedback_.DistanceToGoal = distance_to_goal;
                      feedback_.ElapsedTime = elapsed_time;
                      as_->publishFeedback(feedback_);
                    }
                    if(as_->isPreemptRequested())
                    {
                      robot.cancelGoal();
                      result.Achieved = false;
                      result.DistanceCovered = static_cast<int>(distance_covered);
                      result.ElapsedTime = elapsed_time;
                      as_->setPreempted(result, "Action Preempted");
                      ROS_WARN("Preempt action is requested. Exploration finished");
                    }
                    loop_rate.sleep();
                  }

                  // Check how to get the output of movebase...
                  if(robot.status() == robot.SUCCEEDED)
                  {
                    result.Achieved = true;
                    result.DistanceCovered = static_cast<int>(distance_covered);
                    result.ElapsedTime = elapsed_time;
                    as_->setSucceeded(result, "Action Completed");
                  } else if (robot.status() == robot.ERROR)
                  {
                    result.Achieved = false;
                    result.DistanceCovered = static_cast<int>(distance_covered);
                    result.ElapsedTime = elapsed_time;
                    as_->setAborted(result, "Action Aborted");
                  } else
                  {
                    ROS_WARN("Exploration loop finished without proper condition.");
                    result.Achieved = false;
                    result.DistanceCovered = static_cast<int>(distance_covered);
                    result.ElapsedTime = elapsed_time;
                    as_->setAborted(result, "Action Aborted");
                  }
              }
          }
          else
            ROS_WARN("Couldn't get robot position!");
    }
    else
        ROS_INFO_ONCE("Waiting for first map");
}


} /* cam_exploration */



/**
 * @brief Brings up the node
 */
int main(int argc, char** argv)
{
    ROS_INFO("Exploration Server");
    ros::init(argc, argv, "exploration_server");
    ros::NodeHandle nh_;

    cam_exploration::robot.init();

    ActionServer as(nh_, "exploration_server", boost::bind(&cam_exploration::ExecuteAction, _1, &as), false);

    as.start();

    ros::Rate main_loop_rate(2);

    while(ros::ok()){
      ros::spin();
      main_loop_rate.sleep();
    }

    return 0;
}

