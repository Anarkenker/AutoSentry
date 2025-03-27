#include <opencv2/opencv.hpp>
// #include <opencv2/core.hpp>
// #include <opencv2/core/base.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
#include <bits/stdc++.h>
#include <thread>
#include <queue>
#include <map>
#include <unordered_map>
#include <vector>
#include <fstream>

#include <pcl/io/pcd_io.h>
// #include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <bits/stl_function.h>

#include "tools.h"

using namespace std;
using namespace cv;

constexpr int scale = 4;
constexpr int picSize = 1024 / scale;
int pointcnt[picSize][picSize];
Mat img = Mat::zeros(picSize, picSize, CV_8UC1);
Mat imgShow = Mat::zeros(picSize, picSize, CV_8UC3);

optional<pair<int, int>> startP, endP;

void click(int event, int y, int x, int flags, void *userdata)
{
    if (event == EVENT_LBUTTONDOWN)
    {
        // img.copyTo(imgShow);
        cvtColor(img, imgShow, COLOR_GRAY2BGR);
        startP = {x, y};
        endP = nullopt;
        drawCircle(imgShow, x, y, 3, {0, 0, 255});

        imshow("img", imgShow);

        cout << "x y " << x << ',' << y << endl;

        auto [orgx, orgy] = invtrans(x, y);

        cout << "org x y " << orgx << ',' << orgy << endl;
    }
    else if (event == EVENT_LBUTTONUP)
    {
        endP = {x, y};
        drawCircle(imgShow, x, y, 5, {0, 255, 0});
        // cout << "x y " << y << ',' << x << endl;
        cout << "theta:" << atan2(y - startP->second,x - startP->first) << endl;

        imshow("img", imgShow);
    }
    else if (event == EVENT_MOUSEMOVE && startP && !endP)
    {
        img.copyTo(imgShow);
        line(imgShow, {startP->second, startP->first}, {y, x}, 0x3f, 3);
        imshow("img", imgShow);
    }
}

// using PointType = pcl::PointXYZINormal;
using PointType = pcl::PointXYZ;

int main(int argc, char **argv)
{
    pcl::PointCloud<PointType>::Ptr target_cloud(new pcl::PointCloud<PointType>);
    pcl::io::loadPCDFile<PointType>("../../RMUL.pcd", *target_cloud);

    cout << target_cloud->points.size() << endl;
    initPCD(target_cloud , picSize);

    for (auto p : target_cloud->points)
    {
        if (p.z > 1.5)
            continue;
        auto [x, y] = trans(p.x, p.y);
        pointcnt[x][y]++;
    }

    for (int i = 0; i < picSize; i++)
        for (int j = 0; j < picSize; j++)
        {
            if (pointcnt[i][j] < 3)
                continue;

            img.at<uchar>(i, j) = 0xff;
        }
    
    resize(img,img,{},scale,scale,INTER_NEAREST);
    imwrite("../obstacle.png", img);

    // {
    //     auto [x, y] = trans(0, 0);
    //     cout << "x y " << x << ' ' << y << endl;
    //     drawSquare(img, x, y, 4);
    //     tie(x, y) = invtrans(x, y);
    //     cout << "org x y " << x << ' ' << y << endl;
    // }

    imshow("img", img);

    setMouseCallback("img", click);

    waitKey(0);

    return 0;
}