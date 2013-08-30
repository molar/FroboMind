/*
 * rabbitplannermarker.cpp
 *
 *  Created on: May 4, 2012
 *      Author: morl
 */
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

visualization_msgs::Marker marker;
visualization_msgs::Marker line;

geometry_msgs::Point line_start,line_end;

ros::Publisher p;
ros::Subscriber s;




void spin(const ros::TimerEvent& e)
{
	static tf::StampedTransform optimus_prime;
	static tf::TransformListener decepticon;


	marker.header.frame_id = "rabbit";
	marker.id = 0;
	p.publish(marker);

	marker.header.frame_id = "wpA";
	marker.id = 1;
	p.publish(marker);

	marker.header.frame_id = "wpB";
	marker.id = 2;
	p.publish(marker);

	try
	{
		decepticon.lookupTransform("wpA","wpB",ros::Time(0),optimus_prime);

		line.header.frame_id = "wpA";
		line.type = visualization_msgs::Marker::LINE_STRIP;


		geometry_msgs::Point p1;

		p1.x = 0;
		p1.y = 0;

		line.points.clear();
		line.points.push_back(p1);

		tf::Point p2  = optimus_prime.getOrigin();

		p1.x = p2.x();
		p1.y = p2.y();

		line.points.push_back(p1);

		p.publish(line);

	}
	catch (tf::TransformException ex){
		ROS_DEBUG("Could not find transform between wpA and wpB");
	}








}

int main(int argc,char** argv)
{
	  ros::init(argc, argv, "rabbitplan_marker_node");
	  ros::NodeHandle nh("~");
	  ros::NodeHandle n;

	  ros::Timer t;
	  std::string sub_topic,pub_topic,frame_id;


	  marker.type = visualization_msgs::Marker::CUBE;
	  marker.action = visualization_msgs::Marker::ADD;
	  marker.scale.x = 0.5;
	  marker.scale.y = 0.5;
	  marker.scale.z = 0.5;
	  marker.id = 0;
	  marker.ns = "rabbit_marker";
	  marker.color.a = 0.5;
	  marker.color.g = 1.0;

	  line.type = visualization_msgs::Marker::LINE_STRIP;
	  line.action = visualization_msgs::Marker::ADD;
	  line.scale.x = 0.4;
	  line.scale.y = 0;
	  line.scale.z = 0;
	  line.id = 3;
	  line.ns = "rabbit_marker";
	  line.color.a = 0.5;
	  line.color.b = 1.0;

	  nh.param<std::string>("marker_publisher_topic",pub_topic,"/fmTools/rabbit_markers");

	  marker.header.frame_id = frame_id;

	  p = nh.advertise<visualization_msgs::Marker>(pub_topic.c_str(),10);

	  t = nh.createTimer(ros::Duration(0.05),&spin);

	  ros::spin();

}
