#include "control.h"
#include "roate_fix.h"
#include <iostream>
#include <cmath>
using namespace std;
socket_server<ReceiveNavigationInfo> ser(11456, get_rotate);

void sendControl(double angle, double speed)
{
    double x = cos(angle) * speed;
    double y = sin(angle) * speed;
    SendNavigationInfo pkg{};
    pkg.header = 0x37;
    pkg.vx = x;
    pkg.vy = y;
    ser.send<SendNavigationInfo>(pkg);
}