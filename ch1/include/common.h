#ifndef PCP_MYPCASOLVER_COMMON_H
#define PCP_MYPCASOLVER_COMMON_H

// std
#include <iostream>
#include <cstdlib>
#include <algorithm>
#include <memory>
#include <mutex>
#include <utility>
#include <string>
#include <vector>
#include <string>
#include <fstream>
#include <map>
//#include <unordered_set>
//#include <unordered_map>

// Eigen3
#include <Eigen/Core>
#include <Eigen/Dense>

// typedef
typedef Eigen::Vector3d Vec3;
typedef Eigen::VectorXd VecX;
typedef Eigen::MatrixXd MatX;

typedef std::vector<Eigen::VectorXd, Eigen::aligned_allocator<Eigen::VectorXd>> VecVectorXd;
typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVector3d;
//typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> VecVector2d;

#endif //PCP_MYPCASOLVER_COMMON_H
