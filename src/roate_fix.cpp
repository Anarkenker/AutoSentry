#include "socket_server.hpp"
#include <iostream>
using namespace std;

struct ReceiveGimbalInfo
{
    uint8_t header;
    float yaw;
    float pitch;
} __attribute__((packed));
double yaw;

void get_rotate(const ReceiveGimbalInfo& pkg)
{
    yaw = pkg.yaw;
    cout << "receive yaw: " << yaw << endl;
}

socket_server<ReceiveGimbalInfo> ser(11455, get_rotate);