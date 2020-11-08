#pragma once
#ifndef PCP_MYPCASOLVER_PCASOLVER_H
#define PCP_MYPCASOLVER_PCASOLVER_H

#include "common.h"
#include "pointcloud.h"

class PCASolver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<PCASolver> Ptr;

    // constructor
    PCASolver() = default;
    PCASolver(const PointCloud::Ptr &pointcloud);

    // function member
    // get eigen values
    VecX GetEigenValues() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        return eigen_values_;
    }

    // get eigen vectors
    MatX GetEigenVectors() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        return eigen_vectors_;
    }

    // get data number
    int GetDataNumber() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        return data_number_;
    }

    // get data dimension
    int GetDataDim() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        return data_dim_;
    }

    // get data
    MatX GetDataMat() {
        std::unique_lock<std::mutex> lck(data_mutex_);

        return dataMat_;
    }

    // encoder: compress vector to principle space
    VecX Encoder(const VecX &vec) const;

    // decoder: reconstruct vector from principle space
    VecX Decoder(const VecX &vec) const;

private:
    // normalize data by center (mean)
    void normalize();
    // convert VecVectorXd dataVec_ to MatX dataMat_
    void dataVec2dataMat();
    // sort eigen vectors with eigen values
    void sortByEigenValues();

    // data member
    std::mutex data_mutex_;
    VecVectorXd dataVec_;
    MatX dataMat_;
    VecX eigen_values_;
    MatX eigen_vectors_;

    int data_number_ = 0;
    int data_dim_ = 0;
};
#endif //PCP_MYPCASOLVER_PCASOLVER_H
