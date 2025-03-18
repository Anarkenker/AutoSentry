#include "socket_server.hpp"
#include "roate_fix.h"
#include <iostream>
using namespace std;

struct ReceiveGimbalInfo
{
    uint8_t header;
    float yaw;
    float pitch;
} __attribute__((packed));
double curyaw;
double inityaw = 10;

void get_rotate(const ReceiveGimbalInfo& pkg)
{
    if (inityaw == 10)
    {
        inityaw = pkg.yaw;
    }
    curyaw = pkg.yaw;
    // cout << "receive yaw: " << curyaw << endl;
}

socket_server<ReceiveGimbalInfo> ser(11455, get_rotate);

pcl::PointXYZ fixRotate(pcl::PointXYZ point)
{
    double x = point.x;
    double y = point.y;
    double rad = curyaw - inityaw;
    point.x = x * cos(rad) - y * sin(rad);
    point.y = x * sin(rad) + y * cos(rad);
    return point;
}