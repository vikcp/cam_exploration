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
//TODO: CHECK IF IT IS POSSIBLE THAT THERE IS NO GOAL AT THE END OF THE WHILE...
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
/**/
cam_exploration::ExplorationServerResult setResult(bool achieved, float dist, ros::Duration time)
{
  cam_exploration::ExplorationServerResult result;
  result.Achieved = achieved;
  result.DistanceCovered = static_cast<int>(dist);
  result.ElapsedTime = time;
  return result;
}

/**
 * @brief Node's main loop
 */
void ExecuteAction(const cam_exploration::ExplorationServerGoalConstPtr goal, ActionServer* as_)
{
    string node_ns = ros::this_node::getNamespace();
    string node_name = ros::this_node::getName();
    string node_id = node_ns + node_name;
    if(goal->ExplorationGoal){
      as_->acceptNewGoal();
    } else {
      ROS_WARN("[%s]: Goal not accepted.", node_id.c_str());
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

        ROS_INFO("[%s]: Processing %s", node_id.c_str(), it->c_str());
        if (n_replan.getParam(it->c_str(), parameters)){
            replaner.addCause(it->c_str(), parameters);
        }
        else{
            ROS_INFO("[%s]: No parameters found for replaning cause %s", node_id.c_str(), it->c_str());
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
            ROS_ERROR("[%s]: String %s does not name a valid goal selector", node_id.c_str(), g_selector_name.c_str());
        }
    }
    else
        ROS_ERROR("[%s]: Parameter goal_selector has not been configured", node_id.c_str());



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
      ROS_INFO_ONCE("[%s]: First map received!", node_id.c_str());

          if(robot.refreshPose())
          {
              if(replaner.replan() || first_time)
              {
                  if (first_time)
                      first_time = false;

                  // TODO: How to check is a valid frontier is available...
                  robot.printStatus();
                  if (robot.goTo(decideGoal())) // If the move_base goal is not correctly send, abort.
                  {
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
                            as_->setPreempted(setResult(false, distance_covered, elapsed_time), "Action Preempted");
                            ROS_WARN("[%s]: Preempt action is requested. Exploration finished", node_id.c_str());
                        }
                        loop_rate.sleep();
                    }

                    // Check how to get the output of movebase...
                    if(robot.status() == robot.SUCCEEDED)
                    {
                      as_->setSucceeded(setResult(true, distance_covered, elapsed_time), "Action Completed");
                    } else if (robot.status() == robot.ERROR)
                    {
                      as_->setAborted(setResult(false, distance_covered, elapsed_time), "Action Aborted");
                    } else
                    {
                      ROS_WARN("[%s]: Exploration loop finished without proper condition.", node_id.c_str());
                      as_->setAborted(setResult(false, distance_covered, elapsed_time), "Action Aborted");
                    }

                  }else
                  {
                      ros::Duration no_time = ros::Time::now() - ros::Time::now();
                      as_->setAborted(setResult(false, 0.0, no_time), "Action Aborted");
                  }

              }
          }
          else
            ROS_WARN("[%s]: Couldn't get robot position!", node_id.c_str());
    }
    else
        ROS_INFO_ONCE("[%s]: Waiting for first map", node_id.c_str());
}


} /* cam_exploration */



/**
 * @brief Brings up the node
 */
int main(int argc, char** argv)
{

    ros::init(argc, argv, "exploration_server");
    ros::NodeHandle nh_;

    string node_ns = ros::this_node::getNamespace();
    string node_name = ros::this_node::getName();
    string node_id = node_ns + node_name;
    ROS_INFO("[%s]: Exploration Server Starts", node_id.c_str());

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

