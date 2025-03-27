#include <iostream>
#include <string>

// PCL 相关头文件
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>  // 用于 fromPCLPointCloud2

// VTK 相关头文件
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyDataPointSampler.h>

// 自定义函数：将 pcl::PolygonMesh 转换为 vtkPolyData
vtkSmartPointer<vtkPolyData> convertPolygonMeshToVtkPolyData(const pcl::PolygonMesh &mesh)
{
    // 将 PCLPointCloud2 转换为 pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    // 将点云数据复制到 vtkPoints 中
    vtkSmartPointer<vtkPoints> vtk_points = vtkSmartPointer<vtkPoints>::New();
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        const pcl::PointXYZ &pt = cloud->points[i];
        vtk_points->InsertNextPoint(pt.x, pt.y, pt.z);
    }

    // 构造多边形（面数据）
    vtkSmartPointer<vtkCellArray> vtk_polys = vtkSmartPointer<vtkCellArray>::New();
    for (size_t i = 0; i < mesh.polygons.size(); ++i)
    {
        const pcl::Vertices &vertices = mesh.polygons[i];
        vtkIdType npts = static_cast<vtkIdType>(vertices.vertices.size());
        vtk_polys->InsertNextCell(npts);
        for (size_t j = 0; j < vertices.vertices.size(); ++j)
        {
            vtk_polys->InsertCellPoint(vertices.vertices[j]);
        }
    }

    // 构造 vtkPolyData
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(vtk_points);
    polyData->SetPolys(vtk_polys);

    return polyData;
}

int main(int argc, char** argv)
{
    // 检查输入参数
    if (argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " input.obj [output.pcd]" << std::endl;
        return -1;
    }
    
    std::string input_filename = argv[1];
    std::string output_filename = "sampled_output.pcd";
    if (argc >= 3)
        output_filename = argv[2];

    // 1. 从 OBJ 文件加载网格，使用 loadOBJFile 替换 loadPolygonFileOBJ
    pcl::PolygonMesh mesh;
    if (pcl::io::loadOBJFile(input_filename, mesh) == -1)
    {
        std::cerr << "Failed to load OBJ file: " << input_filename << std::endl;
        return -1;
    }
    std::cout << "Loaded OBJ file: " << input_filename << std::endl;

    // 2. 将 pcl::PolygonMesh 转换为 vtkPolyData
    vtkSmartPointer<vtkPolyData> polyData = convertPolygonMeshToVtkPolyData(mesh);

    // 3. 使用 vtkPolyDataPointSampler 对网格表面进行采样
    vtkSmartPointer<vtkPolyDataPointSampler> sampler = vtkSmartPointer<vtkPolyDataPointSampler>::New();
    sampler->SetInputData(polyData);
    // 设置采样间距（值越小，采样越密集），根据需要调整此值
    sampler->SetDistance(50);
    sampler->Update();

    vtkSmartPointer<vtkPolyData> sampledPolyData = sampler->GetOutput();
    std::cout << "Number of points after sampling: " 
              << sampledPolyData->GetPoints()->GetNumberOfPoints() << std::endl;

    // 4. 将采样后的 vtkPolyData 转换为 pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    vtkSmartPointer<vtkPoints> vtk_points = sampledPolyData->GetPoints();
    for (vtkIdType i = 0; i < vtk_points->GetNumberOfPoints(); i++)
    {
        double p[3];
        vtk_points->GetPoint(i, p);
        pcl::PointXYZ point;
        point.x = static_cast<float>(p[0]) / 1000;
        point.y = static_cast<float>(p[1]) / 1000;
        point.z = static_cast<float>(p[2]) / 1000;
        cloud->points.push_back(point);
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;

    // 5. 保存采样后的点云到 PCD 文件
    if (pcl::io::savePCDFileBinaryCompressed(output_filename, *cloud) == -1)
    {
        std::cerr << "Failed to save PCD file: " << output_filename << std::endl;
        return -1;
    }
    std::cout << "Saved sampled point cloud to: " << output_filename << std::endl;

    return 0;
}
