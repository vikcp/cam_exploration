
/*
 * MIT License
 *
 * Copyright (c) 2016 Jordi Soler Busquets
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "cam_exploration/RobotMotion.h"


/**
 * @file RobotMotion.cpp
 * @brief Implementation of the RobotMotion.h file
 * @author Jordi Soler
 * @version 1
 * @date 2016-04-21
 */

namespace cam_exploration {

// Static members
geometry_msgs::Pose RobotMotion::robot_pose;
geometry_msgs::Pose RobotMotion::prev_robot_pose;
geometry_msgs::Pose RobotMotion::current_goal_;
geometry_msgs::Point RobotMotion::current_frontier_target_point_;
float travelled_distance;
ros::Time start_time;

int RobotMotion::robot_status, RobotMotion::number_of_goals_reached, RobotMotion::number_of_goals_sent;


// Initialize object (not in constructor in order to allow delayed ros::init() in the caller)
void RobotMotion::init()
{
    if(!initialised){
	ac_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>("move_base", true);
	listener = new tf::TransformListener();
	travelled_distance = 0;
	start_time = ros::Time::now();
	map_frame = listener->resolve(map_frame);
	footprint_frame = listener->resolve(footprint_frame);
    }
    else
    {
    	ROS_WARN("RobotMotion: Attempt to initialise RobotMotion when it was already initialised");
    }

}


// Send goal to move_base
bool RobotMotion::goTo(const geometry_msgs::Pose& goal_pose)
{
    ROS_INFO("RobotMotion: Sending goal (%f, %f)", goal_pose.position.x, goal_pose.position.y);
    //tell the action client that we want to spin a thread by default
    //MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    if(!ac_->waitForServer(ros::Duration(5.0)))
    {
	ROS_INFO("RobotMotion: Waiting for the move_base action server to come up");
	return false;
    }

    // overwrite the frame_id and stamp
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = map_frame;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = goal_pose;

    number_of_goals_sent++;

    current_goal_ = goal_pose;

    ac_->sendGoal(goal,
	    boost::bind(&RobotMotion::move_baseDone, this, _1, _2),
	    boost::bind(&RobotMotion::move_baseActive, this));
    robot_status = MOVING; //moving
    return true;
}


//move_baseDone: called when the robot reaches the goal, or navigations finishes for any reason
void RobotMotion::move_baseDone(const actionlib::SimpleClientGoalState& state,  const move_base_msgs::MoveBaseResultConstPtr& result)
{
    ROS_INFO("move_baseDone");
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
	robot_status = SUCCEEDED;
	number_of_goals_reached++;
    }
    else
	robot_status = ERROR;
}


// Called when the robot is set to be active
void RobotMotion::move_baseActive()
{
    robot_status = MOVING;
}

// Print robot status
void RobotMotion::printStatus()
{
    ROS_INFO("Ellapsed time: %f, Travelled distance: %f", (ros::Time::now() - start_time).toSec(), travelled_distance);
}


// Refresh the position of the robot
bool RobotMotion::refreshRobotPosition()
{
    static bool first=true;
    tf::StampedTransform transform;
    ros::Time target_time = ros::Time(0); //ros::Time::now();
    std::string source_frame = map_frame;
    std::string target_frame = footprint_frame;
    try
    {
	if(listener->waitForTransform(source_frame, target_frame, target_time, ros::Duration(5.0)))
	    listener->lookupTransform(source_frame, target_frame, target_time, transform);
	else
	{
	    ROS_ERROR("refreshRobotPosition: no transform between frames %s and %s", source_frame.c_str(), target_frame.c_str());
	    return false;
	}
    }
    catch(tf::TransformException ex)
    {
	ROS_ERROR("%s",ex.what());
	return false;
    }
    robot_pose.position.x = transform.getOrigin().x();
    robot_pose.position.y = transform.getOrigin().y();
    robot_pose.position.z = 0.0;
    robot_pose.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(transform.getRotation()));
    //ROS_INFO("base_link|map: x:%fm y:%fm th:%fº", robot_pose.position.x, robot_pose.position.y, tf::getYaw(robot_pose.orientation));

    if(first)
    {
	prev_robot_pose=robot_pose;
	first=false;
    }

    float dist_incr=std::sqrt(std::pow(prev_robot_pose.position.x-robot_pose.position.x,2)+std::pow(prev_robot_pose.position.y-robot_pose.position.y,2));
    float angle_incr=fabs(tf::getYaw(prev_robot_pose.orientation)-tf::getYaw(robot_pose.orientation));
    float d=0.228; //half of the distance between wheels
    travelled_distance += (fabs(dist_incr+d*angle_incr)+fabs(dist_incr-d*angle_incr))/2.0f;

    prev_robot_pose=robot_pose;
    return true;
}


} // namespace cam_exploration


