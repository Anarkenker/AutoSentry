#include "lidar.h"
#include <atomic>

using namespace std;
mutex lidar_mtx;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket *data, void *client_data)
{
	if (data == nullptr)
	{
		return;
	}

	if (data->data_type == kLivoxLidarCartesianCoordinateHighData)
	{
		// cout << "high data" << endl;
		LivoxLidarCartesianHighRawPoint *p_point_data = (LivoxLidarCartesianHighRawPoint *)data->data;
		lock_guard l(lidar_mtx);
		for (uint32_t i = 0; i < data->dot_num; i++)
		{
			if (p_point_data[i].x == 0 && p_point_data[i].y == 0 && p_point_data[i].z == 0)
				continue;
			pcl::PointXYZ p;
			p.x = p_point_data[i].x / 1000.;
			p.y = p_point_data[i].y / 1000.;
			p.z = p_point_data[i].z / 1000.;
			if (hypot(p.x, p.y) < 0.25)
				continue;
            // p.z = -p.z;
            // p.y = -p.y;
			cloud->push_back(fixRotate(p));
		}
	}
}

void initLidar(const string& path)
{
	DisableLivoxSdkConsoleLogger();

	// REQUIRED, to init Livox SDK2
	if (!LivoxLidarSdkInit(path.c_str()))
	{
		printf("Livox Init Failed\n");
		LivoxLidarSdkUninit();
		return;
	}

	SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
}

void resetPointCloud()
{
    cloud->clear();
}

pcl::PointCloud<pcl::PointXYZ> getPointCloud()
{
	pcl::PointCloud<pcl::PointXYZ> res;
	lidar_mtx.lock();
	res = *cloud;
	cloud->clear();
	lidar_mtx.unlock();
	return res;
}