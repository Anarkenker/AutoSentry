#pragma once

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

void sendCtrl(double yaw,double v);
void initCtrl();
