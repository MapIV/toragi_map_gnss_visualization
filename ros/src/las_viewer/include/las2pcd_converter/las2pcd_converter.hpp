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
 * las2pcd_converter.hpp
 * Author MapIV Aoki Takanose
 */

#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <liblas/liblas.hpp> 

class Las2PcdConverter
{
	public:
		Las2PcdConverter();
		void Convert(std::string las_file);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Downsampling(double leafsize);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,double leafsize);   

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPcdCloud();
    void getInitPosition(double* init_position);

	private:
		bool is_first_;
    double init_position_[3];

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr Downsampling_(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud,double leafsize);
};

Las2PcdConverter::Las2PcdConverter()
{
	// Initialization
	is_first_ = true;
	cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Las2PcdConverter::getPcdCloud()
{
	return cloud_;
}

void Las2PcdConverter::getInitPosition(double* init_position)
{
  // init_position = init_position_;
  init_position[0] = init_position_[0];
  init_position[1] = init_position_[1];
  init_position[2] = init_position_[2];
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Las2PcdConverter::Downsampling(double leafsize)
{
	return Downsampling_(cloud_,leafsize);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Las2PcdConverter::Downsampling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, double leafsize)
{
  return Downsampling_(in_cloud,leafsize);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Las2PcdConverter::Downsampling_(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, double leafsize)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	vg.setInputCloud(in_cloud);
	vg.setLeafSize(leafsize, leafsize, leafsize);
	vg.filter(*out_cloud);
	return out_cloud;
}

void Las2PcdConverter::Convert(std::string las_file)
{
	std::ifstream ifs(las_file, std::ios::in | std::ios::binary);
	liblas::ReaderFactory reader_factory;
	liblas::Reader reader = reader_factory.CreateWithStream(ifs);

	while (reader.ReadNextPoint())
	{
			liblas::Point const& p = reader.GetPoint();

			if(is_first_)
			{
					init_position_[0] = p.GetX();
					init_position_[1] = p.GetY();
					init_position_[2] = p.GetZ();
					is_first_ = false;
			}

			pcl::PointXYZRGB pcl_point;
			pcl_point.x = p.GetX();
			pcl_point.y = p.GetY();
			pcl_point.z = p.GetZ();

			// Extract RGB information
			pcl_point.r = p.GetColor().GetRed();
			pcl_point.g = p.GetColor().GetGreen();
			pcl_point.b = p.GetColor().GetBlue();

			uint16_t r1, g1, b1;
			int r2, g2, b2;
			r1 = p.GetColor().GetRed();
			g1 = p.GetColor().GetGreen();
			b1 = p.GetColor().GetBlue();
			r2 = ceil(((float)r1 / 65536) * (float)256);
			g2 = ceil(((float)g1 / 65536) * (float)256);
			b2 = ceil(((float)b1 / 65536) * (float)256);
			pcl_point.r = r2;
			pcl_point.g = g2;
			pcl_point.b = b2;

			cloud_->push_back(pcl_point);
	}
}
