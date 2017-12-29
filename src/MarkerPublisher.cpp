
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

#include "cam_exploration/MarkerPublisher.h"
#include "cam_exploration/MapServer.h"

#include "tf/transform_datatypes.h"


/**
 * @file MarkerPublisher.cpp
 * @brief Implementation of the MarkerPublisher.h file
 * @author Jordi Soler
 * @version 1.0
 * @date 2016-04-21
 */

namespace cam_exploration {

// namespace cam_exploration--------------------------------
// 		Class pub
//------------------------------------------------------------
pub::pub(const char* name, const char* topic, ros::NodeHandle n, int type)
{
    name_ = name;
    topic_ = topic;
    n_ = n;

    visualization_msgs::Marker m;
    m.header.frame_id = listener_.resolve("map");
    m.type = type;
    m.scale.x = m.scale.y = m.scale.z = 0.07;
    m.action = visualization_msgs::Marker::ADD;
    m.lifetime = ros::Duration(0);
    m.color.a = 1.0;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;

    m_ = m;

    initialise();
}

void pub::initialise()
{
    p_ = n_.advertise<visualization_msgs::Marker>(topic_, 5);
}

void pub::publish( geometry_msgs::Point p, int yaw) const
{
    geometry_msgs::Quaternion q;
    geometry_msgs::Pose pos;
    tf::quaternionTFToMsg (tf::createQuaternionFromRPY(0.0,0.0,yaw), q);
    pos.position = p;
    pos.orientation = q;
    publish_(pos);
}


void pub::publish(const std::vector<int> & cells) const
{
    MapServer mserver;
    std::vector<geometry_msgs::Point> points(cells.size());
    for (std::vector<int>::const_iterator it = cells.begin(); it != cells.end(); it++){
	points.push_back(mserver.cell2point(*it));
    }
    publish_(points);
}


void pub::publish_( geometry_msgs::Pose p ) const
{
    visualization_msgs::Marker m = m_;
    m.header.stamp = ros::Time::now();
    m.pose = p;
    p_.publish(m);
}
void pub::publish_( const std::vector<geometry_msgs::Point>& points) const
{
    std_msgs::ColorRGBA color;
    if (m_.color.a == 0){
	color.a = 1.0;
	color.r = 0.0;
	color.g = 1.0;
	color.b = 0.0;
    }
    else
    {
    	color = m_.color;
    }

    std::vector<std_msgs::ColorRGBA> c(points.size(), color);

    visualization_msgs::Marker m = m_;

    m.header.stamp = ros::Time::now();
    m.ns = name_;

    if (m.type == visualization_msgs::Marker::SPHERE)
	m.type = visualization_msgs::Marker::POINTS;
    m.pose.position.x = m.pose.position.y = 0.0; m.pose.position.z = 0;
    m.pose.orientation.x = m.pose.orientation.y = m.pose.orientation.z = 0; m.pose.orientation.w = 1;
    m.points = points;
    //m.color.g = m.color.a = 0;
    m.colors = c;

    p_.publish(m);
}


//------------------------------------------------------------
// 		Class MarkerPublisher
//------------------------------------------------------------
std::vector<pub> MarkerPublisher::pubs;

MarkerPublisher::MarkerPublisher()
{
    initialise();
}

MarkerPublisher::MarkerPublisher(ros::NodeHandle n_in) : n(n_in)
{
    initialise();
}

void MarkerPublisher::initialise()
{
    marker.header.stamp = ros::Time();
    marker.ns = "MarkerPublisher";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
}


void MarkerPublisher::setNh(ros::NodeHandle n_in)
{
    n = n_in;
}


void MarkerPublisher::publish(geometry_msgs::Point position, const char* topic, const char* frame)
{
    visualization_msgs::Marker m = marker;
    m.pose.position = position;
    ros::Publisher pub = n.advertise<visualization_msgs::Marker>(topic, 10);
    m.header.frame_id = frame;

    pub.publish(m);
}


void MarkerPublisher::add(const char* name, const char* topic, int type)
{
    pub p(name, topic, n, type);
    pubs.push_back(p);
    ROS_INFO("MarkerPublisher: Advertised topic %s", topic);
}

bool MarkerPublisher::has(const char* name) const {
    for(std::vector<pub>::iterator it = pubs.begin(); it != pubs.end(); ++it){
	if (it->is(name)){
	    return true;
	}
    }
    return false;
}

} // namespace cam_exploration


