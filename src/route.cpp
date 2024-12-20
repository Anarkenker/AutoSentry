#include "route.h"
#include "config.h"
#include "tools.h"
using namespace std;
using namespace cv;

Mat img;

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

float heuristic(Point a, Point b)
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

// A*算法实现
vector<Point> astar(Point start, Point end)
{
    priority_queue<Status, vector<Status>, greater<Status>> openSet;

    set<Point, myless> vis;
    map<Point, Point, myless> from;

    openSet.emplace(start, 0, heuristic(start, end));

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

        // 邻居方向
        vector<Point> neighbors = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

        for (const auto &dir : neighbors)
        {
            Point neighbor = current.pos + dir;

            // 确保邻居在图像边界内且可通行
            if (neighbor.x < 0 || neighbor.x >= img.cols ||
                neighbor.y < 0 || neighbor.y >= img.rows ||
                img.at<uchar>(neighbor) == 255 ||
                vis.count(neighbor))
            {
                continue;
            }
            vis.insert(neighbor);

            float c = 1;
            if (abs(dir.x) + abs(dir.y) == 2)
                c = 1.4;
            if (img.at<uchar>(neighbor) == 0x3f)
                c += 15;

            Point last = from[current.pos];
            last = current.pos - last;
            if (last != dir)
                c += 8;

            float gCost = current.g + c; // 假设每一步的成本为1
            float hCost = heuristic(neighbor, end);

            openSet.emplace(neighbor, gCost, hCost);
            from[neighbor] = current.pos;
        }
    }

    return {}; // 如果没有路径
}

function<double(int, double)> funx;
function<double(int, double)> funy;
function<double(double)> cubx;
function<double(double)> cuby;
vector<double> U;
vector<array<double, 3>> X_arg, Y_arg;

int m;

void buildpath(vector<int> x, vector<int> y)
{
    U.clear();
    X_arg.clear();
    Y_arg.clear();

    m = x.size();
    U.resize(m + 1);
    X_arg.resize(m + 1);
    Y_arg.resize(m + 1);
    x.insert(x.begin(), 0);
    y.insert(y.begin(), 0);
    U[1] = 0;
    for (int i = 2; i <= m; i++)
        U[i] = U[i - 1] + hypot(x[i] - x[i - 1], y[i] - y[i - 1]);

    for (int i = 2; i < m; i++)
    {
        Eigen::Matrix3d mat;
        mat << U[i - 1] * U[i - 1], U[i - 1], 1,
            U[i] * U[i], U[i], 1,
            U[i + 1] * U[i + 1], U[i + 1], 1;

        mat = mat.inverse().eval();

        Eigen::Vector3d vx(x[i - 1], x[i], x[i + 1]);
        Eigen::Vector3d vy(y[i - 1], y[i], y[i + 1]);

        auto abcx = (mat * vx).eval();
        X_arg[i][0] = abcx.x();
        X_arg[i][1] = abcx.y();
        X_arg[i][2] = abcx.z();

        auto abcy = (mat * vy).eval();
        Y_arg[i][0] = abcy.x();
        Y_arg[i][1] = abcy.y();
        Y_arg[i][2] = abcy.z();
    }

    funx = [&](int n, double u)
    {
        auto [a, b, c] = X_arg[n];
        return a * u * u + b * u + c;
    };
    funy = [&](int n, double u)
    {
        auto [a, b, c] = Y_arg[n];
        return a * u * u + b * u + c;
    };

    constexpr double eps = 1e-6;
    cubx = [&](double u)
    {
        int n = upper_bound(U.begin(), U.end(), u) - U.begin() - 1;
        if (n <= 2)
            return funx(2, u);
        if (n >= m - 1)
            return funx(m - 1, u);
        return (u - U[n]) / (U[n + 1] - U[n]) * funx(n + 1, u) + (U[n + 1] - u) / (U[n + 1] - U[n]) * funx(n, u);
    };

    cuby = [&](double u)
    {
        int n = upper_bound(U.begin(), U.end(), u) - U.begin() - 1;
        if (n <= 2)
            return funy(2, u);
        if (n >= m - 1)
            return funy(m - 1, u);
        return (u - U[n]) / (U[n + 1] - U[n]) * funy(n + 1, u) + (U[n + 1] - u) / (U[n + 1] - U[n]) * funy(n, u);
    };
}

double getUmax()
{
    return U[m];
}

pair<double, double> calc(double u)
{
    return {cubx(u), cuby(u)};
}

double route_planning(Point start, Point end)
{
    swap(start.x, start.y);
    swap(end.x, end.y);
    // cout << "running A*: (" << start.x << "," << start.y << ") -> (" << end.x << "," << end.y << ")" << endl;
    clock_t s = clock();
    vector<Point> path = astar(start, end);
    cout << "a star use time" << (clock() - s) * 1. / CLOCKS_PER_SEC << '\t';

    cout << "find path length " << path.size() << '\t';

    if (path.size() < 2)
        return 0;
    
    Mat imgShow;

    cvtColor(img, imgShow, COLOR_GRAY2BGR);

    vector<int> endPointx, endPointy;

    endPointx.push_back(path[0].x);
    endPointy.push_back(path[0].y);

    for (int i = 2; i + 1 < path.size(); i++)
    {
        auto cur_dir = path[i] - path[i - 1];
        auto last_dir = path[i - 1] - path[i - 2];
        if (cur_dir == last_dir)
        {
            continue;
        }

        endPointx.push_back(path[i - 1].x);
        endPointy.push_back(path[i - 1].y);
    }
    endPointx.push_back(path.back().x);
    endPointy.push_back(path.back().y);

    // 在图像上绘制路径
    for (const auto &p : path)
    {
        for (int i = -1; i <= 1; i++)
            for (int j = -1; j <= 1; j++)
                imgShow.at<Vec3b>(p.y + i,p.x + j) = Vec3b(0, 255, 0);
    }
    vector<int> lineX, lineY;
    for (int i = 1; i < endPointx.size(); i++)
    {
        auto x1 = endPointx[i - 1], x2 = endPointx[i];
        auto y1 = endPointy[i - 1], y2 = endPointy[i];
        lineX.push_back(x1);
        lineY.push_back(y1);
        if (hypot(x2 - x1, y2 - y1) < 0.1)
            continue;
        for (double theta = 0.4; theta <= 0.61; theta += 0.2)
        {
            // double theta = 0.5;
            lineX.push_back(x2 * theta + x1 * (1 - theta) + 0.01);
            lineY.push_back(y2 * theta + y1 * (1 - theta) + 0.007);
        }
        // lineX.push_back(x2); lineY.push_back(y2);
    }
    lineX.push_back(endPointx.back());
    lineY.push_back(endPointy.back());

    buildpath(lineX, lineY);
    // buildpath(endPointx, endPointy);

    double umax = U.back();

    vector<Point2f> curved_path;

    // cout << "curved:\n";
    optional<pair<double,double>> res;
    // int cnt2 = 0;
    // for (double u = 0; u <= umax; u += umax / 3000)
    // {
    //     auto [x, y] = calc(u);
    //     curved_path.emplace_back(x, y);
    //     // cout << x << ' ' << y << '\n';
    //     int X = round(x);
    //     int Y = round(y);
    //     if (X < 0 || X > picSize || Y < 0 || Y > picSize)
    //         continue;
    //     if (++cnt2 == 10)
    //         *res = {x, y};
    //     int n = upper_bound(U.begin(), U.end(), u) - U.begin() - 1;
    //     for (int i = -1; i <= 1; i++)
    //         for (int j = -1; j <= 1; j++)
    //             imgShow.at<Vec3b>({X + i, Y + j}) = Vec3b(n % 2 ? 0 : 255, 0, n % 2 ? 255 : 0);
    // }
    drawSquare(imgShow, endPointy[0],endPointx[0],2);
    drawSquare(imgShow, endPointy[1],endPointx[1],2);
    static int cnt;
    if (cnt++ % 10 == 0)
        imwrite("../a.png",imgShow);
    
    // cout << endPointy[1] << ' ' << endPointy[0]<< ' ' <<endPointx[1] << ' ' << endPointx[0] << '\t';

    return atan2(endPointx[1] - endPointx[0],endPointy[1] - endPointy[0]);
}

void initRoute(const string &obstaclePath)
{
    img = imread(obstaclePath, 0);
}