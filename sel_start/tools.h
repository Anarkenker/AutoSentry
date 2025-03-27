#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <chrono>

std::pair<int, int> trans(double x, double y);

std::pair<double, double> invtrans(double x, double y);
void drawCircle(cv::Mat &img, int x, int y, int r, cv::Vec3b color);
void initPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr ptr, int picSize);
std::chrono::_V2::system_clock::time_point clock_start();
double get_clock_time();
int meter2pixel(double meter);