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
 * las_viewer_node.cpp
 * Author MapIV Aoki Takanose
 */


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <las2pcd_converter/las2pcd_converter.hpp>


int main(int argc, char** argv)
{
	// ROS NODE initialization
	ros::init(argc, argv, "las_viewer_node");
	ros::NodeHandle nh;
	ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/las_viewer/pointcloud", 1);

	std::string las_file;
	nh.getParam("las_viewer_node/las_file", las_file);
	std::cout << "las_file: " << las_file << std::endl;

	double leafsize;
	nh.getParam("las_viewer_node/leafsize", leafsize);
	std::cout << "leafsize: " << leafsize << std::endl;

	// // Argument handling
	// std::vector<std::string> las_files;
	// for(int i=1; i<argc; i++)
	// {
	// 	las_files.push_back(argv[i]);
	// 	std::cout << "input file: " << argv[i] << std::endl;
	// }

	std::istringstream ss(las_file);
	std::string token;
	std::vector<std::string> las_files;
	while (std::getline(ss, token, ' ')) {
			las_files.push_back(token);
	}

	int file_num = las_files.size();

	// Convert from Las file to pcd format
	Las2PcdConverter las2pcd_converter;
	for(int i=0; i<file_num; i++)
	{
		las2pcd_converter.Convert(las_files[i]);
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = las2pcd_converter.getPcdCloud();
	cloud = las2pcd_converter.Downsampling(cloud,leafsize);

	// Creating ROS Messages
	sensor_msgs::PointCloud2 ros_pc;
	pcl::toROSMsg(*cloud, ros_pc);
	ros_pc.header.frame_id = "map";
	ros_pc.header.stamp = ros::Time::now();

	// Creating ROS TF
	double init_position[3]; // The origin of the local coordinates is the first point cloud position
	las2pcd_converter.getInitPosition(init_position);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(init_position[0], init_position[1], init_position[2]));
  q.setRPY(0, 0, 0);
  transform.setRotation(q);


	while (ros::ok())
	{
			pointcloud_pub.publish(ros_pc);
			br.sendTransform(tf::StampedTransform(transform, ros_pc.header.stamp, ros_pc.header.frame_id, "world"));
			ros::spinOnce();
			ros::Duration(1.0).sleep();
	}
}
