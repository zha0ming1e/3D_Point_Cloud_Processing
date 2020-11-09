#pragma once
#ifndef PCP_MYPCASOLVER_VOXELGRIDFILTER_H
#define PCP_MYPCASOLVER_VOXELGRIDFILTER_H

#include "common.h"
#include "pointcloud.h"

class VoxelGridFilter {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VoxelGridFilter> Ptr;

    // ALGO
    enum class ALGO { CENTROID, RANDOM };

    // constructor
    VoxelGridFilter() = default;
    VoxelGridFilter(const PointCloud::Ptr &origin_pointcloud_ptr,
                    Vec3 voxel_grid_size,
                    ALGO algorithm);

    // function member
    // get original point cloud
    PointCloud::Ptr GetOriginPC() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        return origin_pointcloud_ptr_;
    }

    // get filtered point cloud
    PointCloud::Ptr GetFilteredPC() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        return filtered_pointcloud_ptr_;
    }

    // get voxel grid size
    Vec3 GetVoxelGridSize() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        return voxel_grid_size_;
    }

    // get algorithm
    ALGO GetAlgo() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        return algorithm_;
    }

    // get algorithm in string
    std::string GetAlgoStr() {
        //
        std::string algo_str;
        if (algorithm_ == ALGO::CENTROID)
            algo_str = "Centroid";
        else if (algorithm_ == ALGO::RANDOM)
            algo_str = "Random";

        return algo_str;
    }

    // change voxel grid size
    void ChangeVoxelGridSize(const Vec3 &voxel_grid_size) {
        //
        std::cout << "[ VoxelGridFilter Changing Voxel Grid Size... ]" << std::endl;

        PointCloud::Ptr ptr(new PointCloud());
        filtered_pointcloud_ptr_ = ptr;

        voxel_grid_size_ = voxel_grid_size;

        // voxel grid filtering
        voxel_grid_filter();

        std::cout << "[ VoxelGridFilter Change Voxel Grid Size Finished. ]" << std::endl;
    }

    // change algorithm
    void ChangeAlgo(ALGO algorithm) {
        //
        std::cout << "[ VoxelGridFilter Changing Algorrithm... ]" << std::endl;

        PointCloud::Ptr ptr(new PointCloud());
        filtered_pointcloud_ptr_ = ptr;

        algorithm_ = algorithm;

        // voxel grid filtering
        voxel_grid_filter();

        std::cout << "[ VoxelGridFilter Change Algorithm Finished. ]" << std::endl;
    }

    // output VoxelGridFilter Info
    std::string Info() {
        //
        std::string info = "\n[ VoxelGridFilter Info: ] ";
        // output info of origin and filtered point cloud
        if (origin_pointcloud_ptr_ && filtered_pointcloud_ptr_) {
            auto OrigPCNum = origin_pointcloud_ptr_->GetPointsNum();
            auto FiltPCNum = filtered_pointcloud_ptr_->GetPointsNum();
            std::string algo_str = GetAlgoStr();
            info += "\n==> VoxelGridFilter Algorithm: " + algo_str;
            info += "\n==> Num of Original Point Cloud = " + std::to_string(OrigPCNum);
            info += "\n==> Num of Filtered Point Cloud = " + std::to_string(FiltPCNum);
            info += "\n==> [ Filter Ratio = " + std::to_string((double) FiltPCNum / (double) OrigPCNum * 100.0) + " % ] \n";
        } else {
            info += "\nWarning: Original or Filtered Point Cloud is empty. \n";
        }

        return info;
    }

private:
    // private function member
    // return -> std::vector(2) -> { (minX, minY, minZ), (maxX, maxY, maxZ)}
    VecVector3d minmaxVertex(const PointCloud::Ptr &point_cloud_ptr);
    void filter_algorithm(const std::map<long long, int> &index_and_id);
    void voxel_grid_filter();

    // data member
    std::mutex data_mutex_;
    PointCloud::Ptr origin_pointcloud_ptr_ = nullptr;
    PointCloud::Ptr filtered_pointcloud_ptr_ = nullptr;
    Vec3 voxel_grid_size_;
    ALGO algorithm_ = ALGO::CENTROID;
};
#endif //PCP_MYPCASOLVER_VOXELGRIDFILTER_H
