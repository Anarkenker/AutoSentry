#include "route.h"
#include "config.h"
#include "tools.h"
using namespace std;
using namespace cv;

Mat img;
Mat imgFixed;

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

bool isObstacle(Point u)
{
    Scalar res = img.at<Vec3b>(u.x, u.y);
    return res[0] || res[1] || res[2];
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
        if (filterObstacle && isObstacle(neighbor))
            continue;
        neighbors.push_back(neighbor);
    }
    return neighbors;
}

int costMap[picSize][picSize];
void setTarget(Point target)
{
    auto timeStart = chrono::system_clock::now();
    imgFixed.copyTo(img);
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
    if (isObstacle(end))
        return {};

    if (isObstacle(start))
    {
        for (int dt = 1; dt <= 3; dt++)
        {
            for (int di = -dt; di <= dt; di++)
                for (int dj = -dt; dj <= dt; dj++)
                {
                    if (!isObstacle({start.x + di, start.y + dj}))
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

            float gCost = current.g + 1; // 假设每一步的成本为1
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
        if (isObstacle({x0, y0}))
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

void addObstacle(deque<pcl::PointXYZ> &obstacle)
{
    map<pair<int, int>, int> pointcnt;
    mtx.lock();
    cout << obstacle.size() << '\t';
    for (int i = 0; i < obstacle.size(); i++)
    {
        auto p = obstacle[i];
        if (p.z > 0.6)
            continue;
        auto [x, y] = trans(p.x, p.y);
        pointcnt[{x, y}]++;
    }
    mtx.unlock();

    for (auto &[p, cnt] : pointcnt)
    {
        if (cnt < 2)
            continue;
        auto [x, y] = p;
        drawCircle(img, x, y, 20, {0, 255, 255});
    }
}

double route_planning(Point start, Point end, deque<pcl::PointXYZ> &realtimeObstacle)
{
    // swap(start.x, start.y);
    // swap(end.x, end.y);
    // cout << "running A*: (" << start.x << "," << start.y << ") -> (" << end.x << "," << end.y << ")" << endl;
    imgFixed.copyTo(img);

    addObstacle(realtimeObstacle);
    clock_t s = clock();
    vector<Point> path = astar(start, end);

    cout << "a star use time" << (clock() - s) * 1. / CLOCKS_PER_SEC << '\t' << flush;

    if (path.size() < 2)
    {
        cout << "not find path\t" << flush;
        imwrite("../not find path.png", img);
        return std::nan("");
    }

    vector<Point> fixedPath;
    auto cur = path.front();
    fixedPath.push_back(cur);
    while (cur != path.back())
    {
        bool fg = false;
        for (auto it = path.rbegin(); it != path.rend(); it++)
        {
            if (line_of_sight(cur, *it))
            {
                fixedPath.push_back(*it);
                cur = *it;
                fg = true;
                break;
            }
        }
        if (fg)
            continue;
        cout << "fix failed\t" << flush;
        fixedPath.push_back(path.back());
        break;
    }
    path = fixedPath;

    cout << "find path length " << path.size() << '\t' << flush;

    // 在图像上绘制路径
    auto lastp = path.front();
    swap(lastp.x, lastp.y);
    for (auto p : path)
    {
        swap(p.x, p.y);
        line(img, lastp, p, {0, 255, 0}, 3);
        lastp = p;
    }

    drawCircle(img, path[0].x, path[0].y, 2, {255, 255, 0});
    drawCircle(img, path[1].x, path[1].y, 2, {0, 255, 255});
    static int cnt;
    if (cnt++ % 10 == 0)
        imwrite("../a.png", img);

    return atan2(path[1].y - path[0].y, path[1].x - path[0].x);
}

void initRoute(const string &obstaclePath)
{
    imgFixed = imread(obstaclePath);
}