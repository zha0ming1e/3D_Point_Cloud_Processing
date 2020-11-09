#pragma once
#ifndef PCP_MYPCASOLVER_POINT_H
#define PCP_MYPCASOLVER_POINT_H

#include "common.h"

class Point {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Point> Ptr;

    // constructor
    Point() = default;
    Point(const Vec3 &position);
    Point(const Vec3 &position, const Vec3 &normal);

    // function member
    // get 3D position
    Vec3 GetPos() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        return pos_;
    }

    // set 3D position
    void SetPos(const Vec3 &pos) {
        std::unique_lock<std::mutex> lck(data_mutex_);

        pos_ = pos;
    }

    // get normal of point
    Vec3 GetNormal() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        return normal_;
    }

    // set normal of point
    void SetNormal(const Vec3 &normal) {
        std::unique_lock<std::mutex> lck(data_mutex_);

        normal_ = normal;
    }

    // factory function
    static Point::Ptr CreateNewPoint();

private:
    // data member
    std::mutex data_mutex_;
    Vec3 pos_ = Vec3::Zero();
    Vec3 normal_ = Vec3::Zero();
};
#endif //PCP_MYPCASOLVER_POINT_H
