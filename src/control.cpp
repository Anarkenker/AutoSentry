#include "control.h"
#include "roate_fix.h"
#include <iostream>
#include <cmath>
using namespace std;
socket_server<ReceiveGimbalInfo> ser(11455, get_rotate);

void sendControl(double angle, double speed)
{
    double x = sin(angle) * speed;
    double y = cos(angle) * speed;
    SendNavigationInfo pkg{};
    pkg.header = 6b;
    pkg.vx = x;
    pkg.vy = y;
    ser.send<SendNavigationInfo>(pkg);
}