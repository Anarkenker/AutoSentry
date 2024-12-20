#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <optional>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


void initRoute(const std::string &obstaclePath);
double route_planning(cv::Point start, cv::Point end, std::deque<pcl::PointXYZ>& realtimeObstacle);
