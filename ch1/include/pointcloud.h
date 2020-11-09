#pragma once
#ifndef PCP_MYPCASOLVER_POINTCLOUD_H
#define PCP_MYPCASOLVER_POINTCLOUD_H

#include "common.h"
#include "point.h"

class PointCloud {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<PointCloud> Ptr;
    typedef std::vector<Point::Ptr> PointCloudType;
    // typedef std::unordered_set<Point::Ptr> PointCloudType;

    // constructor
    PointCloud() = default;
    PointCloud(const std::string &filename);

    // function member
    // read points from a txt file: append == false -> read data from a file, not append from a file
    void From_File_TXT(const std::string &filename, bool append=false);

    // get all points
    PointCloudType GetAllPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        return points_;
    }

    // get points number
    unsigned long GetPointsNum() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        return points_num_;
    }

    // insert a 3D point
    void InsertPoint(const Point::Ptr &point);

    // clear point cloud
    void Clear() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        points_ = PointCloudType();
        points_num_ = 0;
    }

private:
    // data member
    std::mutex data_mutex_;
    PointCloudType points_;
    unsigned long points_num_ = 0;
};

#endif //PCP_MYPCASOLVER_POINTCLOUD_H
