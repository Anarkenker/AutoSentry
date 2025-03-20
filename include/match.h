#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <string>

Eigen::Matrix4f& match(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, double& res);

void initMatch(std::string globalPointCloudPath, double approX, double approY, double approTheta);