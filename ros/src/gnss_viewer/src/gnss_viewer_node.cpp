// Copyright (c) 2023, Map IV, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of the Map IV, Inc. nor the names of its contributors
//   may be used to endorse or promote products derived from this software
//   without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/*
 * nmea2pose_node.cpp
 * Author MapIV Aoki Takanose
 */


#include "ros/ros.h"
#include "nmea2pose/sentence2gpgga.hpp"
#include "nmea2pose/llh2JPRCS.hpp"

#include "nmea_msgs/Sentence.h"
#include "nmea_msgs/Gpgga.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"


class Nmea2PoseNode
{
public:
  Nmea2PoseNode(ros::NodeHandle& nh) : nh_(nh)
  {

		std::string nmea_sentence_topic_name;

		nh_.getParam("gnss_viewer_node/nmea_sentence_topic_name", nmea_sentence_topic_name);
		nh_.getParam("gnss_viewer_node/plane_num", plane_num_);
		nh_.getParam("gnss_viewer_node/update_distance", update_distance_);
		std::cout << "nmea_sentence_topic_name: " << nmea_sentence_topic_name << std::endl;
		std::cout << "plane_num: " << plane_num_ << std::endl;
		std::cout << "update_distance: " << update_distance_ << std::endl;

    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/gnss/pose", 1000);
    nmea_sentence_sub_ = nh_.subscribe(nmea_sentence_topic_name, 1000, &Nmea2PoseNode::NmeaSentenceCallback, this);
  }
  void run()
  {
    ros::spin();
  }

private:
  // ROS
  ros::NodeHandle nh_;
  ros::Publisher pose_pub_;
  ros::Subscriber nmea_sentence_sub_;

	int plane_num_;
	double update_distance_;
	double last_plane_x, last_plane_y, last_yaw;

  void NmeaSentenceCallback(const nmea_msgs::Sentence::ConstPtr& msg)
  {	
    nmea_msgs::Sentence sentence = *msg;
		nmea_msgs::Gpgga gpgga;
		if(!nmea2gpgga_converter(sentence, &gpgga)) return;

		double lat_rad = gpgga.lat *M_PI/180;
		double lon_rad = gpgga.lon *M_PI/180;
		double height  = gpgga.alt;
		double plane_x, plane_y, plane_z;
		llh2JPRCS_converter(lat_rad, lon_rad, height, plane_x, plane_y,  plane_z, plane_num_);

		double yaw, distance;
		distance = std::sqrt(std::pow(plane_x - last_plane_x, 2) + std::pow(plane_y - last_plane_y, 2));
		if(distance >= update_distance_)
		{
			yaw = atan2(plane_y-last_plane_y, plane_x-last_plane_x);
			last_plane_x = plane_x;
			last_plane_y = plane_y;
			last_yaw = yaw;
		}
		else
		{
			yaw = last_yaw;
		}
		geometry_msgs::Quaternion quat;
		tf::Quaternion tf_quat = tf::createQuaternionFromRPY(0, 0, yaw);
		quaternionTFToMsg(tf_quat, quat);

		geometry_msgs::PoseStamped pose;
		pose.header = msg->header;
		pose.header.frame_id = "map";
		pose.pose.position.x = plane_x;
		pose.pose.position.y = plane_y;
		pose.pose.position.z = plane_z;
		pose.pose.orientation = quat;
		pose_pub_.publish(pose);

		// Creating ROS TF
		tf::Transform transform;
		tf::Quaternion q;
		transform.setOrigin(tf::Vector3(plane_x, plane_y, plane_z));
		q.setRPY(0, 0, yaw);
		transform.setRotation(q);
		static tf::TransformBroadcaster br;
		br.sendTransform(tf::StampedTransform(transform, pose.header.stamp, "map", "gnss"));
  }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gnss_viewer_node");
  ros::NodeHandle nh;

  Nmea2PoseNode node(nh);
  node.run();

  return 0;
}