#include <bits/stdc++.h>
#include "lidar.h"
#include "match.h"
#include "tools.h"
#include "route.h"
#include "control.h"
#include "config.h"
#include "socket_server.hpp"
#include "logger.h"
#include "game_status.h"

using namespace std;
using namespace cv;
// int nx = 386;
// int ny = 581;
// double theta = -0.318748;
// int nx = 185;
// int ny = 266;
// double theta = 1.62171;
int nx = 98;
int ny = 91;
double theta = 0.;

Point home = {nx, ny};
Point center = {475,348};

double getfps()
{
    static thread_local deque<std::chrono::_V2::system_clock::time_point> q;
    auto cur = clock_start();
    q.push_back(cur);

    while (std::chrono::duration_cast<std::chrono::milliseconds>(cur - q.front()).count() > 1000)
        q.pop_front();
    if (q.size() == 1)
        return 0;
    return q.size() / (std::chrono::duration_cast<std::chrono::milliseconds>(cur - q.front()).count() / 1000.);
}

double x, y, t, score;
deque<pcl::PointXYZ> realtimeObstacle;
double locateFPS;
int matchPointNum = 30000;
int obstacleNum = 60000;
void locate()
{
    while (true)
    {
        static pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
        auto p = getPointCloud();
        for (auto pt : p.points)
        {
            pointCloud->push_back(pt);
        }
        if (pointCloud->size() > matchPointNum)
        {
            pointCloud->erase(pointCloud->begin(), pointCloud->begin() + pointCloud->size() - matchPointNum);
        }
        if (pointCloud->empty())
            continue;
        Eigen::Matrix4f& transMat = match(pointCloud, score);
        mtx.lock();
        for (auto pt : p.points)
        {
            double dis = hypot(pt.x, pt.y, pt.z);
            // if (dis > 2)
            //     continue;
            Eigen::Vector4f transPoint{pt.x, pt.y, pt.z, 1};
            transPoint = transMat * transPoint;
            realtimeObstacle.push_back({transPoint[0], transPoint[1], transPoint[2]});
        }
        while (realtimeObstacle.size() > obstacleNum)
            realtimeObstacle.pop_front();
        mtx.unlock();
        x = transMat(0, 3);
        y = transMat(1, 3);
        t = atan2((transMat(1, 0) - transMat(0, 1)) / 2, (transMat(0, 0) + transMat(1, 1)) / 2) + getDeltaYaw();
        locateFPS = getfps();
        // this_thread::sleep_for(10ms);
    }
}


void endCtrl(int sig)
{
    sendControl(0, 0);
    exit(0);
}
double rotate = 0;
extern bool showImg;
int main(int argc, char const *argv[])
{
    if (argc > 1)
    {
        showImg = true;
    }
    // signal(SIGINT, endCtrl);
    // signal(SIGTERM, endCtrl);
    // signal(SIGSEGV, endCtrl);
    initLidar("../mid360_config.json");

    // this_thread::sleep_for(5s);
    // auto ptr = getPointCloud();
    // cout << ptr->size() << endl;
    // pcl::io::savePCDFileBinaryCompressed("d.pcd", *ptr);
    // return 0;

    initRoute("../RMUL.png");
    initMatch("../RMUL.pcd", nx, ny, theta);
    // initRoute("../1floor.png");
    // initMatch("../1floor.pcd", nx, ny, theta);
    cout << "finish init" << endl;
    // this_thread::sleep_for(5s);
    cout << fixed << setprecision(4);
    auto startTime = chrono::steady_clock::now();
    // resetPointCloud();

    
    setTarget(center);
    thread _{locate};
    auto curDest = new Point(center);

    int stableCnt = 0;

    while (true)
    {
        // startTime += 100ms;
        // this_thread::sleep_until(startTime);
        log_info(hp, start_status);
        log_info(score, x, y, t);

        Point start;
        tie(start.x, start.y) = trans(x, y);

        auto endPoint = *curDest;
        double dt = route_planning(start, endPoint, realtimeObstacle);
        auto realEnd = invtrans(endPoint.x, endPoint.y);
        double dis = hypot(realEnd.first - x, realEnd.second - y);

        double ddt = dt - t;
        if (abs(ddt) > M_PI)
        {
            if (ddt > 0)
                ddt = 2 * M_PI - ddt;
            else
                ddt = ddt + 2 * M_PI;
        }

        log_info(dis, dt, ddt, getfps(), locateFPS);
        log_new_line();

        // sendControl(1, 1);
        if (!start_status)
            continue;
        
        if (hp < .25)
            *curDest = home;
        else if (hp > .9)
            *curDest = center;

        if (score > .1 || isnan(ddt))
        {
            stableCnt = 0;
            // sendControl(deltaTime / 2 * M_PI, 1);
            continue;
        }
        if (++stableCnt < 10)
        {
            continue;
        }

        sendControl(ddt, 1);

        // this_thread::sleep_for(1s);
    }
}
