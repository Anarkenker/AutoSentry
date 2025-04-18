#include "route.h"
#include "config.h"
#include "tools.h"
#include "logger.h"
using namespace std;
using namespace cv;

Mat imgObs;
Mat imgFixed;
bool showImg = false;

struct Status
{
    Point pos;
    float g;
    float h;

    Status(Point p, float gCost, float hCost)
        : pos(p), g(gCost), h(hCost) {}

    bool operator>(const Status &other) const
    {
        return g + h > other.g + other.h;
    }
};

float distance(Point a, Point b)
{
    return hypot(a.x - b.x, a.y - b.y);
}

struct myless : public binary_function<Point, Point, bool>
{
    _GLIBCXX14_CONSTEXPR
    bool
    operator()(const Point &__x, const Point &__y) const
    {
        return make_pair(__x.x, __x.y) < make_pair(__y.x, __y.y);
    }
};

int getObstacleLevel(Point u)
{
    uchar res = imgObs.at<uchar>(u.x, u.y);
    return res;
}

vector<Point> getNeighbors(Point x, bool filterObstacle = true)
{
    static vector<Point> direction = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};
    vector<Point> neighbors;
    for (auto dir : direction)
    {
        Point neighbor = x + dir;
        if (neighbor.x < 0 || neighbor.x >= picSize || neighbor.y < 0 || neighbor.y >= picSize)
            continue;
        if (filterObstacle && getObstacleLevel(neighbor) == 0xff)
            continue;
        neighbors.push_back(neighbor);
    }
    return neighbors;
}

int costMap[picSize][picSize];
void setTarget(Point target)
{
    auto timeStart = chrono::system_clock::now();
    fill(&costMap[0][0], &costMap[0][0] + picSize * picSize, 1e9);

    costMap[target.x][target.y] = 0;
    queue<pair<Point, int>> q;
    q.push({target, 0});
    while (q.size())
    {
        auto [x, c] = q.front();
        q.pop();

        for (auto neighbor : getNeighbors(x))
        {
            if (costMap[neighbor.x][neighbor.y] < 1e9)
                continue;

            costMap[neighbor.x][neighbor.y] = c + 1;
            q.push({neighbor, c + 1});
        }
    }
    cout << "set target use time:" << chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now() - timeStart).count() << "ms\t" << flush;
}

// A*算法实现
vector<Point> astar(Point start, Point end)
{
    auto timeStart = chrono::system_clock::now();
    if (getObstacleLevel(end) == 255)
        return {};

    if (getObstacleLevel(start) == 255)
    {
        for (int dt = 1; dt <= meter2pixel(1); dt++)
        {
            for (int di = -dt; di <= dt; di++)
                for (int dj = -dt; dj <= dt; dj++)
                {
                    Point neighbor = start + Point{di, dj};
                    if (neighbor.x < 0 || neighbor.x >= picSize || neighbor.y < 0 || neighbor.y >= picSize)
                        continue;
                    if (getObstacleLevel(neighbor) < 255)
                    {
                        start.x += di;
                        start.y += dj;
                        goto finish;
                    }
                }
        }
        return {};
    }
finish:;

    priority_queue<Status, vector<Status>, greater<Status>> openSet;

    set<Point, myless> vis;
    map<Point, Point, myless> from;

    openSet.emplace(start, 0, costMap[start.x][start.y]);

    while (!openSet.empty())
    {
        Status current = openSet.top();
        openSet.pop();

        // 如果到达目标
        if (current.pos == end)
        {
            Point pos = current.pos;
            vector<Point> path;
            while (pos != start)
            {
                path.push_back(pos);
                pos = from[pos];
            }
            path.push_back(pos);
            reverse(path.begin(), path.end());
            return path;
        }

        for (auto neighbor : getNeighbors(current.pos))
        {
            // 确保邻居在图像边界内且可通行
            if (vis.count(neighbor))
            {
                continue;
            }
            vis.insert(neighbor);

            float gCost = current.g + getObstacleLevel(neighbor);
            float hCost = costMap[neighbor.x][neighbor.y];

            openSet.emplace(neighbor, gCost, hCost);
            from[neighbor] = current.pos;
        }
    }

    return {}; // 如果没有路径
}

// 判断两点之间是否存在直线可见性
bool line_of_sight(const Point &p1, const Point &p2)
{
    // 使用 Bresenham 算法检测 p1 和 p2 之间是否有障碍物
    int x0 = p1.x, y0 = p1.y;
    int x1 = p2.x, y1 = p2.y;

    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        if (x0 < 0 || y0 < 0 || x0 >= 1024 || y0 >= 1024)
        {
            cout << "error line_of_sight" << flush;
            return false;
        }
        if (getObstacleLevel({x0, y0}))
            return false; // 障碍物
        if (x0 == x1 && y0 == y1)
            break;
        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y0 += sy;
        }
    }
    return true;
}

void drawCircleObstacle(int x, int y, int r, int extr)
{
    r += extr;
    for (int i = -r; i <= r; i++)
        for (int j = -r; j <= r; j++)
        {
            int nx = x + i;
            int ny = y + i;
            if (nx < 0 || ny < 0 || nx >= imgObs.rows || ny >= imgObs.cols)
                continue;

            int curr = hypot(i, j);
            if (curr <= r - extr)
            {
                imgObs.at<uchar>(x + i, y + j) = 0xff;
            }
            else if (curr <= r)
            {
                imgObs.at<uchar>(x + i, y + j) = max((int)imgObs.at<uchar>(x + i, y + j), 2 * uchar(r - curr));
            }
        }
}

void addObstacle(deque<pcl::PointXYZ> &obstacle)
{
    unordered_map<int, int> pointcnt;
    mtx.lock();
    log_info(obstacle.size());
    for (int i = 0; i < obstacle.size(); i++)
    {
        auto p = obstacle[i];
        if (p.z > 0.4)
            continue;
        auto [x, y] = trans(p.x, p.y);
        pointcnt[(x << 16) | y]++;
    }
    mtx.unlock();

    vector<pair<int,int>> vpointcnt(pointcnt.begin(), pointcnt.end());

#pragma omp parallel for
    for (auto &[p, cnt] : vpointcnt)
    {
        if (cnt < 2)
            continue;
        int x = p >> 16;
        int y = p & ((1 << 16) - 1);
        drawCircleObstacle(x, y, meter2pixel(0.35), meter2pixel(0.15));
    }
}

double route_planning(Point start, Point end, deque<pcl::PointXYZ> &realtimeObstacle)
{
    // swap(start.x, start.y);
    // swap(end.x, end.y);
    // cout << "running A*: (" << start.x << "," << start.y << ") -> (" << end.x << "," << end.y << ")" << endl;
    imgFixed.copyTo(imgObs);

    clock_start();
    addObstacle(realtimeObstacle);
    static vector<Point> last_path;
    for (auto p : last_path)
    {
        costMap[p.x][p.y] -= 10;
    }

    vector<Point> path = astar(start, end);

    for (auto p : last_path)
    {
        costMap[p.x][p.y] += 10;
    }

    last_path = path;

    log_info("a star use time", get_clock_time());

    if (path.size() < 2)
    {
        log_info("not find path");
        imwrite("../not find path.png", imgObs);
        if (showImg)
        {
            imshow("a", imgObs);
            waitKey(1);
        }
        return std::nan("");
    }

    vector<Point> fixedPath;
    auto curit = path.begin();
    fixedPath.push_back(*curit);
    while (curit != path.end())
    {
        bool fg = false;
        for (auto it = path.rbegin(); *it != *curit; it++)
        {
            if (line_of_sight(*curit, *it))
            {
                fixedPath.push_back(*it);
                curit = it.base();
                fg = true;
                break;
            }
        }
        if (fg)
        {
            continue;
        }
        fixedPath.push_back(*curit);
        curit++;
    }
    path = fixedPath;

    log_info("find path length ", path.size());

    // 在图像上绘制路径
    auto lastp = path.front();
    swap(lastp.x, lastp.y);
    for (auto p : path)
    {
        swap(p.x, p.y);
        line(imgObs, lastp, p, 200, 1);
        // drawCircle(imgob)
        lastp = p;
    }
    for (auto p : path)
    {
        swap(p.x, p.y);
        imgObs.at<uchar>(p) = 100;
    }
    static int cnt;
    if (cnt++ % 10 == 0)
        imwrite("../a.png", imgObs);
    if (showImg)
    {
        imshow("a", imgObs);
        waitKey(1);
    }
    int nxt_step = 1;
    for (;nxt_step < path.size(); nxt_step++)
    {
        if (hypot(path[nxt_step].y - path[0].y, path[nxt_step].x - path[0].x) > 3)
            break;
    }

    return atan2(path[nxt_step].y - path[0].y, path[nxt_step].x - path[0].x);
}

void initRoute(const string &obstaclePath)
{
    imgFixed = imread(obstaclePath, 0);
    imgFixed.copyTo(imgObs);
}
