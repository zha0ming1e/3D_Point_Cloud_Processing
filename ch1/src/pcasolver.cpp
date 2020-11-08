#include "pcasolver.h"

PCASolver::PCASolver(const PointCloud::Ptr &pointcloud) {
    // check
    auto points = pointcloud->GetAllPoints();
    if (points.empty()) {
        std::cerr << "Error: Point Cloud is empty. " << std::endl;
        return;
    }
    std::cout << "\n>>>>>>>> PCASolver <<<<<<<<"
              << "\n[ 0. PCASolver: Initializing PCASolver... ] from [ PointCloud ]... " << std::endl;

    // feed data
    data_number_ = points.size();
    data_dim_ = points[0]->GetPos().rows();
    for (auto &pt : points) {
        dataVec_.push_back(pt->GetPos());
    }

    // normalize
    std::cout << "[ 1. PCASolver: Normalizing data of PCASolver... ] " << std::endl;
    normalize();
    std::cout << "[ 1. Normalization finished. ] " << std::endl;

    // construct dataMat_
    dataVec2dataMat();

    // compute eigen values and vectors
    auto dataMat = GetDataMat();
    auto covMat = dataMat * dataMat.transpose();

    Eigen::EigenSolver<MatX> eigenSolver(covMat);
    eigen_values_ = eigenSolver.eigenvalues().real();
    eigen_vectors_ = eigenSolver.eigenvectors().real();

    // sort
    std::cout << "[ 2. PCASolver: Sorting... ] " << std::endl;
    sortByEigenValues();
    std::cout << "[ 2. Sort finished. ] " << std::endl;

    // finish initialization
    std::cout << "[ 0. PCASolver: Initialization finished. ] "
              << "\n>>>>>>>> PCASolver <<<<<<<<" << std::endl;
}

void PCASolver::normalize() {
    // check
    if (dataVec_.empty()) {
        std::cerr << "Error: PCASlover has been not fed with data, data is empty. " << std::endl;
        return;
    }

    // mean
    int num = 0;
    VecX mean = VecX::Zero(data_dim_, 1);
    for (auto &datai : dataVec_) {
        mean += datai;
        ++num;
    }
    mean = 1.0 / num * mean;

    for (auto &datai : dataVec_) {
        datai = datai - mean;
    }
}

void PCASolver::dataVec2dataMat() {
    // convert VecVectorXd dataVec_ to MatX dataMat_
    MatX dataMat = MatX::Zero(data_dim_, data_number_);
    for (int i = 0; i < data_number_; ++i) {
        dataMat.col(i) = dataVec_[i];
    }
    dataMat_ = dataMat;
}

void PCASolver::sortByEigenValues() {
    VecX eigen_values_sorted = VecX::Zero(eigen_values_.rows(), 1);
    MatX eigen_vectors_sorted = MatX::Zero(eigen_vectors_.rows(), eigen_vectors_.cols());

    // std::map -> automatic sorting -> std::greater<...> or std::less<...>
    std::map<double, VecX, std::greater<double>> eigenValuesAndVectors;
    for (int i = 0; i < eigen_values_.size(); ++i)
        eigenValuesAndVectors.insert(std::make_pair((double) eigen_values_[i], eigen_vectors_.col(i)));

//    // sorting
//    std::sort(eigenValuesAndVectors.begin(), eigenValuesAndVectors.end(),
//              [](const std::pair<double, VecX> &pair1, const std::pair<double, VecX> &pair2) -> bool {
//                  return pair1.first > pair2.first;
//              });

    int n = 0;
    for (auto &eigenValuesAndVector : eigenValuesAndVectors) {
        eigen_values_sorted[n] = eigenValuesAndVector.first;
        eigen_vectors_sorted.col(n) = eigenValuesAndVector.second;
        ++n;
    }

    eigen_values_ = eigen_values_sorted;
    eigen_vectors_ = eigen_vectors_sorted;
}

VecX PCASolver::Encoder(const VecX &vec) const {
    if (vec.rows() != eigen_vectors_.rows()) {
        std::cerr << "Error: vector dim and PCASolver data dim not epuals. " << std::endl;

        return VecX();
    }

    return eigen_vectors_.transpose() * vec;
}

VecX PCASolver::Decoder(const VecX &vec) const {
    if (vec.rows() != eigen_vectors_.cols()) {
        std::cerr << "Error: vector dim and PCASolver data dim not epuals. " << std::endl;

        return VecX();
    }

    return eigen_vectors_ * vec;
}
