#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

int main()
{
    // 创建点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取PCD文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../RMUL.pcd", *cloud) == -1)
    {
        PCL_ERROR("无法读取输入文件！\n");
        return -1;
    }

    // 创建绕X轴旋转90度的变换矩阵
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitX()));

    // 应用变换
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

    // 保存结果
    pcl::io::savePCDFileASCII("../RMUL-r.pcd", *transformed_cloud);

    return 0;
}