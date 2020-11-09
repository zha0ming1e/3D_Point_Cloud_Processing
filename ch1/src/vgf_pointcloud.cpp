#include "voxelgridfilter.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>

// function declaration
// PointCloud to PCL Point Cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2PCL(const PointCloud::Ptr &point_cloud_ptr);
///////////////////////////////////////////////////////
// main
int main(int argc, char **argv) {
    // ARGC
    if (argc != 2) {
        std::cerr << "Error: ARGC is not 2. \nUsage: [PROGRAM] [PATH_TO_DATA_FILE] " << std::endl;
        return -1;
    }
    // data file
    std::string filename = argv[1];
    std::ifstream fin(filename);
    if (!fin) {
        std::cerr << "Error: can not open the file: [ " << filename << " ] " << std::endl;
        return -2;
    }
    // PointCloud
    PointCloud::Ptr point_cloud_ptr(new PointCloud(filename));
    // VoxelGridFilter
    // 1. centroid algo
    VoxelGridFilter::Ptr voxel_grid_filter_ptr(new VoxelGridFilter(point_cloud_ptr, Vec3({0.08, 0.08, 0.08}), VoxelGridFilter::ALGO::CENTROID));
    // 2. random algo
    // VoxelGridFilter::Ptr voxel_grid_filter_ptr(new VoxelGridFilter(point_cloud_ptr, Vec3({0.08, 0.08, 0.08}), VoxelGridFilter::ALGO::RANDOM));

    // original point cloud
    auto origin_cloud_ptr = PointCloud2PCL(voxel_grid_filter_ptr->GetOriginPC());
    // filtered point cloud
    auto filtered_cloud_ptr = PointCloud2PCL(voxel_grid_filter_ptr->GetFilteredPC());

    // output info of origin and filtered point cloud
    std::cout << voxel_grid_filter_ptr->Info() << std::endl;

    // visualization of origin point cloud
    pcl::visualization::PCLVisualizer::Ptr viewer0(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer0->addPointCloud<pcl::PointXYZ>(origin_cloud_ptr, "origin point cloud");

    // visualization of filtered point cloud
    pcl::visualization::PCLVisualizer::Ptr viewer1(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer1->addPointCloud<pcl::PointXYZ>(filtered_cloud_ptr, "filtered point cloud");

    viewer0->spin();
    // viewer1->spin();

    return 0;
}
///////////////////////////////////////////////////////

// function definition
// PointCloud to PCL Point Cloud
pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloud2PCL(const PointCloud::Ptr &point_cloud_ptr) {
    // PCL
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PtCloud;
    // origin point cloud
    PtCloud::Ptr pcl_cloud_ptr(new PtCloud());
    auto all_points_num = point_cloud_ptr->GetPointsNum();
    auto all_points = point_cloud_ptr->GetAllPoints();
    for (int i = 0; i < all_points_num; ++i) {
        auto ptr_i = all_points[i];
        PointT pt;
        pt.x = ptr_i->GetPos().x();
        pt.y = ptr_i->GetPos().y();
        pt.z = ptr_i->GetPos().z();
        pcl_cloud_ptr->push_back(pt);
    }

    return pcl_cloud_ptr;
}
