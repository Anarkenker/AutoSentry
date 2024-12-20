#include <bits/stdc++.h>
#include "lidar.h"
#include "match.h"
#include "tools.h"
#include "route.h"
#include "control.h"

using namespace std;
using namespace cv;
int nx = 369;
int ny = 581;
double theta = -0.252242;

vector<Point> dest{
    {616, 447},
    {369, 581}};

int getfps()
{
    static deque<int> q;
    auto cur = clock();
    q.push_back(cur);

    while (cur - q.front() > CLOCKS_PER_SEC)
        q.pop_front();
    if (q.size() == 1)
        return 0;
    return q.size() * CLOCKS_PER_SEC / (cur - q.front());
}

int main()
{
    initLidar("../mid360_config.json");
    initRoute("../obstacle.png");
    initCtrl();
    initMatch("../scans.pcd", nx, ny, theta);
    cout << "finish init" << endl;

    cout << fixed << setprecision(4);
    auto startTime = chrono::steady_clock::now();
    // resetPointCloud();

    auto curDest = dest.begin();

    while (1)
    {
        // startTime += 100ms;
        // this_thread::sleep_until(startTime);
        auto p = getPointCloud(20000);

        double score;
        auto [x, y, t] = match(p, score);

        cout << p->size() << ' ' << x << '\t' << y << '\t' << t << '\t';

        Point start;
        tie(start.x, start.y) = trans(x, y);

        auto endPoint = *curDest;
        double dt = route_planning(start, endPoint);
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

        cout << dis << '\t' << ddt << '\t' << getfps() << endl;

        if (score > .1)
        {
            sendCtrl(.03, 0);
            continue;
        }

        sendCtrl((ddt) / 20., 0);
        if (abs(ddt) < .5)
            sendCtrl(0, 0.5);

        if (dis < .5)
        {
            curDest++;
            if (curDest == dest.end())
                curDest = dest.begin();
        }
        // this_thread::sleep_for(1s);
    }
}
