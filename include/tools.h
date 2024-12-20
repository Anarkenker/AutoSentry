#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

std::pair<int, int> trans(double x, double y);

std::pair<double, double> invtrans(double x, double y);
void drawSquare(cv::Mat &img, int x, int y, int r);
void initPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr ptr, int picSize);