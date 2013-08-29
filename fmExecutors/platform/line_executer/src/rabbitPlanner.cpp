/*
 * rabbitPlanner.cpp
 *
 *  Created on: Jan 1, 2001
 *      Author: soeni05
 */

#include "rabbitPlanner.h"
#include <math.h>
void operator >> (geometry_msgs::Point point, tf::Vector3& vec){
	vec[0] = point.x;
	vec[1] = point.y;
	vec[2] = point.z;
}

rabbitPlanner::rabbitPlanner(std::string name)
	: action_server(nh,name, boost::bind(&rabbitPlanner::actionExecute, this, _1), false),action_name(name)
{

	path = new std::vector<geometry_msgs::PoseStamped>();
	action_server.start();

}

void rabbitPlanner::setParams(int rabbit_type,double distance_scale,double angle_scale,double delta_rabbit,double delta_waypoint)
{
	this->rabbit_type = rabbit_type;
	this->deltaWaypoint = delta_waypoint;
	this->deltaRabbit = delta_rabbit;
	this->angle_scale = angle_scale;
	this->distance_scale = distance_scale;
}


bool rabbitPlanner::planRabbit()
{
	bool ret = false;
	static tf::Vector3 AB; // vector from A to B
	static double ABSquared;
	static tf::Vector3 Abase; // vector from A to Base
	static double rabbitScale; // distance from point to line

	if(locateVehicle())
	{
		//load a and B from plan

		path->at(current_waypoint-1).pose.position >> A;
		path->at(current_waypoint).pose.position >> B;


		// calculate distances
		AB = B-A;
		ABSquared = AB[1]*AB[1]+AB[0]*AB[0];
		Abase = B-base;

		rabbitScale = (Abase[0]*AB[0] + Abase[1]*AB[1])/ABSquared;


		// check if target reached either because it is close enough to B waypoint
		// or if it has passed the waypoint
		if(B.distance(base) < deltaWaypoint)
		{
			ROS_INFO("Switching waypoint due to deltaWaypoint check, %.4f",deltaWaypoint);

			current_waypoint++;
			if(current_waypoint >= path->size())
			{
				// we are done
				ret = true;
			}
			else
			{
				// create new A and B
				path->at(current_waypoint-1).pose.position >> A;
				path->at(current_waypoint).pose.position >> B;
				publishFeedback();
			}

		}
		else if(rabbitScale <= 0)
		{
			ROS_INFO("Switching waypoint due to rabbitScale check, %.4f",rabbitScale);

			current_waypoint++;
			if(current_waypoint >= path->size())
			{
				// we are done
				ret = true;
			}
			else
			{
				// create new A and B
				path->at(current_waypoint-1).pose.position >> A;
				path->at(current_waypoint).pose.position >> B;
				publishFeedback();
			}

		}

		if(!ret)
		{
			// place rabbit
			rabbit = ((B-(rabbitScale * AB)));

			if(rabbit_type == 1){
				rabbit = rabbit + (deltaRabbit)*AB/ABSquared;
			}else if(rabbit_type == 2){
				rabbit = (B-rabbit)/deltaRabbit+rabbit;
			}else if(rabbit_type == 0)
			{



				double target_dist = (base-rabbit).length();

				rabbit = rabbit + (deltaRabbit)*AB.normalize();

				tf::Vector3 direction_vector = rabbit - base;

				double target_orientation = atan2(direction_vector[1],direction_vector[0]);
				while(target_orientation >= M_PI*2 )
				{
					target_orientation -= M_PI*2;
				}

				while(target_orientation < 0 )
				{
					target_orientation += M_PI*2;
				}

				double target_error = target_orientation - tf::getYaw(base_orientation);

				while(target_error < -M_PI)
				{
					target_error += 2*M_PI;
				}

				while(target_error > M_PI)
				{
					target_error -= 2*M_PI;
				}

				//ROS_INFO_THROTTLE(0.5,"base_orientation: %.4f heading: %.4f, err: %.4f",tf::getYaw(base_orientation),target_orientation,target_error);
				ROS_INFO_THROTTLE(0.5,"angular: %.4f linear: %.4f",target_error,target_dist);
				//((double angle_error = atan2(rabbit[1]-base[1],rabbit[0]-base[0]);
				ROS_INFO_THROTTLE(0.5,"angdist: %.4f",target_error*angle_scale + target_dist*distance_scale);
				//ROS_INFO_THROTTLE(0.5,"Rabbit distance error is %.4f angle error is %.4f",distance_error,angle_error);
				//ROS_INFO_THROTTLE(0.5,"A %.4f %.4f B %.4f %.4f \n Base %.4f %.4f Rabbit %.4f %.4f",A[0],A[1],B[0],B[1],base[0],base[1],rabbit[0],rabbit[1]);

				//vel_msg.twist.linear.x = forward_velocity;
				//vel_msg.twist.angular.z = angle_error;
				vel_msg.twist.linear.x = forward_velocity;
				vel_msg.twist.angular.z = target_error*angle_scale + target_dist*distance_scale;
				//vel_msg.header.stamp = ros::Time::now();
				cmd_vel_publisher.publish(vel_msg);


			}else{
				ROS_ERROR("RABBIT WENT BACK TO ITS HOLE! WRONG 'rabbit_type' ");
			}

			if(B.distance(rabbit) < deltaWaypoint)
			{
				rabbit = B;
			}

			publishWPAB();
			publishRabbit();

		}
		else
		{
			place_safe_rabbit();
			publishResult();
		}

	}
	else
	{
		ROS_ERROR_THROTTLE(1,"Could not locate vehicle");
	}

	return ret;
}


void rabbitPlanner::publishPath()
{
	nav_msgs::Path msg;

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = odom_frame;

	for(unsigned int i=current_waypoint; i< path->size();i++)
	{
		msg.poses.push_back(path->at(i));
	}

	path_publisher.publish(msg);

}

void rabbitPlanner::publishFeedback()
{
	ROS_INFO("Goal feedback published");
	feedback_msg.reached_pose_nr = current_waypoint;

	action_server.publishFeedback(feedback_msg);
}

void rabbitPlanner::publishResult()
{
	ROS_INFO("Goal result published");
	geometry_msgs::PoseStamped p;

	p.header.frame_id = odom_frame;
	p.header.stamp = ros::Time::now();
	p.pose.position.x = base.getX();
	p.pose.position.y = base.getY();
	p.pose.position.z = base.getZ();
	tf::quaternionTFToMsg(transform.getRotation(),p.pose.orientation);
	result_msg.resulting_pose = p;
	action_server.setSucceeded(result_msg);
}

void rabbitPlanner::actionExecute(const line_executer::follow_pathGoalConstPtr& goal)
{
	ROS_INFO("Goal received");

	//reset variables
	success = false;
	once = false;
	current_waypoint = 1;

	ros::Rate r(25);

	path->clear();
	for(unsigned int i=0;i<goal->path.poses.size();i++)
	{
		if(goal->path.poses[i].header.frame_id != this->odom_frame)
		{
			geometry_msgs::PoseStamped* ps = new geometry_msgs::PoseStamped();
			try
			{
				tf_listener.transformPose(this->odom_frame,goal->path.poses[i],*ps);
				ROS_DEBUG("ADDING PATH %.4f %.4f",ps->pose.position.x,ps->pose.position.y);
				path->push_back(*ps);
			}
			catch (tf::TransformException& ex){
				ROS_WARN("PATH transform failed %s",ex.what());
			}
		}
		else
		{
			path->push_back(goal->path.poses.at(i));
		}
	}

	if(path->size() == 0)
	{
		geometry_msgs::PoseStamped p;

		p.header.frame_id = odom_frame;
		p.header.stamp = ros::Time::now();
		p.pose.position.x = base.getX();
		p.pose.position.y = base.getY();
		p.pose.position.z = base.getZ();
		tf::quaternionTFToMsg(transform.getRotation(),p.pose.orientation);
		result_msg.resulting_pose = p;
		action_server.setAborted(result_msg,"Path is empty");
		success= true;
	}


	ROS_INFO("Path has %ld entries",path->size());
	publishPath();

	while(!success)
	{
		if(action_server.isPreemptRequested() || !ros::ok())
		{
			ROS_INFO("Goal is preempted");
			// action is preempted
			action_server.setPreempted();
			place_safe_rabbit();
			break;
		}
		else
		{
			success = planRabbit();
		}
		r.sleep();

	}
}

void rabbitPlanner::publishWPAB()
{
	tf_A.header.stamp = ros::Time::now();
	tf_A.header.frame_id = odom_frame.c_str();
	tf_A.child_frame_id = "wpA";

	tf_A.transform.translation.x = A.getX();
	tf_A.transform.translation.y = A.getY();
	tf_A.transform.translation.z = A.getZ();
	tf_A.transform.rotation.x = 0;
	tf_A.transform.rotation.y = 0;
	tf_A.transform.rotation.z = 0;
	tf_A.transform.rotation.w = 1;
	tf_broadcaster.sendTransform(tf_A);

	tf_B.header.stamp = ros::Time::now();
	tf_B.header.frame_id = odom_frame.c_str();
	tf_B.child_frame_id = "wpB";
	tf_B.transform.rotation.x = 0;
	tf_B.transform.rotation.y = 0;
	tf_B.transform.rotation.z = 0;
	tf_B.transform.rotation.w = 1;

	tf_B.transform.translation.x = B.getX();
	tf_B.transform.translation.y = B.getY();
	tf_B.transform.translation.z = B.getZ();
	tf_broadcaster.sendTransform(tf_B);
}

void rabbitPlanner::publishRabbit()
{
	tf_rabbit.header.stamp = ros::Time::now();
	tf_rabbit.header.frame_id = odom_frame.c_str();
	tf_rabbit.child_frame_id = rabbit_frame.c_str();

	tf_rabbit.transform.translation.x = rabbit.getX();
	tf_rabbit.transform.translation.y = rabbit.getY();
	tf_rabbit.transform.translation.z = rabbit.getZ();
	tf_rabbit.transform.rotation.x = 0;
	tf_rabbit.transform.rotation.y = 0;
	tf_rabbit.transform.rotation.z = 0;
	tf_rabbit.transform.rotation.w = 1;

	ROS_DEBUG_THROTTLE(1,"%.4f %.4f %.4f",rabbit.getX(),rabbit.getY(),rabbit.getZ());

	tf_broadcaster.sendTransform(tf_rabbit);
}

bool rabbitPlanner::locateVehicle()
{
	bool ret = false;
	try
	{
		tf_listener.waitForTransform(odom_frame.c_str(),vehicle_frame.c_str(),ros::Time::now(),ros::Duration(2));
		tf_listener.lookupTransform(odom_frame.c_str(),vehicle_frame.c_str(),ros::Time(0), transform);

		base = transform.getOrigin();
		base_orientation = transform.getRotation();
		ret = true;
	}
	catch (tf::TransformException& ex){
		ROS_WARN("%s",ex.what());
	}

	return ret;
}

void rabbitPlanner::place_safe_rabbit()
{
	ROS_INFO("Planning safe rabbit");
	tf_rabbit.header.stamp = ros::Time::now();
	tf_rabbit.header.frame_id = odom_frame.c_str();
	tf_rabbit.child_frame_id = rabbit_frame.c_str();

	try
	{
	tf_listener.lookupTransform(odom_frame.c_str(),vehicle_frame.c_str(),ros::Time(0), transform);
	tf::Vector3 v = transform.getOrigin();
	tf_rabbit.transform.translation.x = v.getX();
	tf_rabbit.transform.translation.y = v.getY();
	tf_rabbit.transform.translation.z = v.getZ();
	tf::quaternionTFToMsg(transform.getRotation(),tf_rabbit.transform.rotation);


	tf_broadcaster.sendTransform(tf_rabbit);
	}
	catch (tf::TransformException& ex){
		ROS_WARN("%s",ex.what());
	}
	vel_msg.twist.linear.x = 0;
	vel_msg.twist.angular.z = 0;
	cmd_vel_publisher.publish(vel_msg);
}



