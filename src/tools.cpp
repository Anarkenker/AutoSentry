#include "tools.h"
using namespace std;

inline double minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
inline double rate;
std::pair<int, int> trans(double x, double y)
{
    return {lround((x - minx) * rate), lround((y - miny) * rate)};
}

std::pair<double, double> invtrans(double x, double y)
{
    return {x / rate + minx, y / rate + miny};
}

void drawCircle(cv::Mat &img, int x, int y, int r, cv::Vec3b color)
{
    for (int i = -r; i <= r; i++)
        for (int j = -r; j <= r; j++)
        {
            int nx = x + i;
            int ny = y + i;
            if (nx < 0 || ny < 0 || nx >= img.rows || ny >= img.cols)
                continue;
            if (hypot(i, j) <= r)
                img.at<cv::Vec3b>(x + i, y + j) = color;
        }
}

void initPCD(pcl::PointCloud<pcl::PointXYZ>::Ptr ptr, int picSize)
{
    for (auto p : *ptr)
    {
        double x, y;
        x = p.x;
        y = p.y;
        minx = std::min(minx, x);
        maxx = std::max(maxx, x);
        miny = std::min(miny, y);
        maxy = std::max(maxy, y);
    }
    // cout << "minx " << minx << " maxx" << maxx << "\nminy" << miny << " maxy" << maxy << endl;

    rate = (picSize - 1) / std::max(maxx - minx, maxy - miny);

    std::cout << 1 / rate << "m per pixel" << std::endl;

}


thread_local std::chrono::_V2::system_clock::time_point start_time;

std::chrono::_V2::system_clock::time_point clock_start()
{
    return start_time = chrono::system_clock::now();
}

double get_clock_time()
{
    return chrono::duration_cast<chrono::duration<double>>(chrono::system_clock::now() - start_time).count();
}