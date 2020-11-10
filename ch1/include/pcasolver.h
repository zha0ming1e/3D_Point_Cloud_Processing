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
    explicit PCASolver(const PointCloud::Ptr &pointcloud);
    explicit PCASolver(const VecVectorXd &dataVec);
    explicit PCASolver(const MatX &dataMat0);

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

    // output PCASolver Info
    std::string Info() {
        //
        std::string info = "\n[ PCASolver Info: ] ";
        // output info
        if (data_number_ && data_dim_) {
            info += "\n==> Data Number = " + std::to_string(data_number_);
            info += "\n==> Data Dimension = " + std::to_string(data_dim_);
            info += "\n==> Eigen Values Number = " + std::to_string(eigen_values_.rows());
            info += "\n==> Eigen Vectors Number = " + std::to_string(eigen_vectors_.cols())
                    + " , Dimension = " + std::to_string(eigen_vectors_.rows()) + " \n";
        } else {
            info += "\nWarning: Data number or dimension is 0. \n";
        }

        return info;
    }

private:
    // normalize data by center (mean)
    void normalize();
    // convert VecVectorXd dataVec_ to MatX dataMat_
    void dataVec2dataMat();
    // convert MatX dataMat_to VecVectorXd dataVec_
    void dataMat2dataVec();
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
