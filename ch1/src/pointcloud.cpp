#include "pointcloud.h"

PointCloud::PointCloud(const std::string &filename) {
    // initialization
    std::cout << "\n[ PointCloud Initializing... ] " << std::endl;
    // clear the existing points
    Clear();
    // read points data from a txt file
    From_File_TXT(filename, false);

    std::cout << "[ PointCloud Initialization finished. ] " << std::endl;
}

void PointCloud::InsertPoint(const Point::Ptr &point) {
    if (std::find(points_.begin(), points_.end(), point) == points_.end()) {
        points_.push_back(point);
        ++points_num_;
    } else {
        std::cout << "\nWarning: the Point Instance has been in the PointCloud. " << std::endl;
    }
}

void PointCloud::From_File_TXT(const std::string &filename, bool append) {
    // txt file: x,y,z,nx,ny,nz
    std::ifstream fin(filename);
    if (!fin) {
        std::cerr << "Error: can not open the file. " << std::endl;
        return;
    }

    if (!append)
        Clear();  // clear the existing points

    double x, y, z, nx, ny, nz;
    char sep;
    while (fin >> x >> sep >> y >> sep >> z >> sep >> nx >> sep >> ny >> sep >> nz) {
        Point::Ptr ptr(new Point(Vec3 (x, y, z), Vec3(nx, ny, nz)));
        points_.push_back(ptr);
        ++points_num_;
    }

    std::cout << "[ PointCloud loaded. ] \n\\==> Total number of points from TXT file = " << points_num_ << std::endl;

    fin.close();
}
