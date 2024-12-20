#pragma once
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <optional>


void initRoute(const std::string &obstaclePath);
double route_planning(cv::Point start, cv::Point end);
