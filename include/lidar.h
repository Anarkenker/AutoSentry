#pragma once

#include <string>
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "roate_fix.h"

void initLidar(const std::string& path);
void resetPointCloud();
pcl::PointCloud<pcl::PointXYZ> getPointCloud();