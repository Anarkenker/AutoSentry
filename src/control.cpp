#include "control.h"
#include "roate_fix.h"
#include <iostream>
#include <cmath>
using namespace std;
socket_server<ReceiveGimbalInfo> ser(11455, get_rotate);

void sendControl(double angle, double speed)
{
    double x = cos(angle) * speed;
    double y = sin(angle) * speed;
    SendNavigationInfo pkg{};
    pkg.header = 0x6b;
    pkg.vx = x;
    pkg.vy = y;
    ser.send<SendNavigationInfo>(pkg);
}