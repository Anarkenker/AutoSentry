#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
struct ReceiveGimbalInfo
{
    uint8_t header;
    float yaw;
    float pitch;
    double hp;
} __attribute__((packed));
pcl::PointXYZ fixRotate(pcl::PointXYZ point);
void get_rotate(const ReceiveGimbalInfo& pkg);
double getDeltaYaw();