#include "pcasolver.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>

// functions: declaration
// calculate principle vectors and visualization
void calPrinVec(const PointCloud::Ptr &point_cloud_ptr,
                const PCASolver::Ptr &pca_solver_ptr);
// project to the first greater principle vectors and visualization
void projToFirst2PrinVec(const PointCloud::Ptr &point_cloud_ptr,
                         const PCASolver::Ptr &pca_solver_ptr);
// calculate each point normal in a point cloud with kd-tree search and visualization
void calPointCloudNormals(const PointCloud::Ptr &point_cloud_ptr);
// calculate normal for a point cloud by PCA (NO VISUALIZATION)
void calPointNormal(const pcl::PointCloud<pcl::PointXYZ> &cloud,
                    pcl::Normal &normal);

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
    // PCASolver
    PCASolver::Ptr pca_solver_ptr(new PCASolver(point_cloud_ptr));

    // TASK 1, 2, 3 and visualization
    // 1. calculate principle vectors
    calPrinVec(point_cloud_ptr, pca_solver_ptr);
    // 2. project to the first 2 greater principle vectors, i.e. the first principle plane
    projToFirst2PrinVec(point_cloud_ptr, pca_solver_ptr);
    // 3. calculate point cloud normals
    calPointCloudNormals(point_cloud_ptr);

    return 0;
}
///////////////////////////////////////////////////////

// functions: definition
//
void calPrinVec(const PointCloud::Ptr &point_cloud_ptr,
                const PCASolver::Ptr &pca_solver_ptr) {
    // calculate principle vectors

    // pcl types
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PtCloud;
    PtCloud::Ptr output(new PtCloud());
    // pcl point cloud
    auto all_points = point_cloud_ptr->GetAllPoints();
    auto pt_num = point_cloud_ptr->GetPointsNum();
    for (int i = 0; i < pt_num; ++i) {
        auto point_pos = all_points[i]->GetPos();

        PointT point;
        point.x = point_pos.x();
        point.y = point_pos.y();
        point.z = point_pos.z();

        output->push_back(point);
    }

    // visualization
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(output, "point cloud");

    auto eigen_vectors = pca_solver_ptr->GetEigenVectors();
    // origin point
    PointT origpt(0, 0, 0);
    PointT vector1(eigen_vectors.col(0).x(), eigen_vectors.col(0).y(), eigen_vectors.col(0).z());
    PointT vector2(eigen_vectors.col(1).x(), eigen_vectors.col(1).y(), eigen_vectors.col(1).z());
    PointT vector3(eigen_vectors.col(2).x(), eigen_vectors.col(2).y(), eigen_vectors.col(2).z());

    viewer->addArrow(vector1, origpt, 1, 0, 0, false, "Eigen Vector 1");
    viewer->addArrow(vector2, origpt, 0, 1, 0, false, "Eigen Vector 2");
    viewer->addArrow(vector3, origpt, 0, 0, 1, false, "Eigen Vector 3");
    viewer->spin();
}

//
void projToFirst2PrinVec(const PointCloud::Ptr &point_cloud_ptr,
                         const PCASolver::Ptr &pca_solver_ptr) {
    // project to the first greater principle vectors

    // pcl types
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PtCloud;
    PtCloud::Ptr output(new PtCloud());
    // pcl point cloud
    auto all_points = point_cloud_ptr->GetAllPoints();
    for (int i = 0; i < point_cloud_ptr->GetPointsNum(); ++i) {
        auto projected_point_pos = pca_solver_ptr->Encoder(all_points[i]->GetPos());

        PointT point;
        point.x = projected_point_pos.x();
        point.y = projected_point_pos.y();
        point.z = 0;

        output->push_back(point);
    }

    // visualization
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(output, "point cloud");
    viewer->spin();
}

//
void calPointCloudNormals(const PointCloud::Ptr &point_cloud_ptr) {
    // calculate each point normal in a point cloud with kd-tree search

    // pcl types
    typedef pcl::PointXYZ PointT;
    typedef pcl::PointCloud<PointT> PtCloud;
    typedef pcl::Normal Normal;
    typedef pcl::PointCloud<pcl::Normal> NormCloud;
    // from PointCloud format to PCL format
    PtCloud::Ptr cloud(new PtCloud());
    auto all_points = point_cloud_ptr->GetAllPoints();
    auto pt_num = point_cloud_ptr->GetPointsNum();
    for (int i = 0; i < pt_num; ++i) {
        auto pos_i = all_points[i]->GetPos();
        PointT pt_i(pos_i.x(), pos_i.y(), pos_i.z());
        cloud->push_back(pt_i);
    }

    // kd-tree
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    // calculate each point normal according to local neighbor point cloud
    int K = 20;
    std::vector<int> ptIdxNKNSearch;
    std::vector<float> ptNKNSquaredDis;
    NormCloud::Ptr cloud_normals(new NormCloud());
    for (int i = 0; i < pt_num; ++i) {
        PointT searchPoint((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
        if (kdtree.nearestKSearch(searchPoint, K, ptIdxNKNSearch, ptNKNSquaredDis) > 0) {
            PtCloud local_neighbors;
            for (auto &idx : ptIdxNKNSearch)
                local_neighbors.push_back((*cloud)[idx]);
            // calculate point normal according to local point neighbors
            Normal normal;
            calPointNormal(local_neighbors, normal);
            cloud_normals->push_back(normal);
        } else {
            cloud_normals->push_back(Normal());
        }
    }

    // visualization
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<PointT>(cloud, "point cloud");
    viewer->addPointCloudNormals<PointT, Normal>(cloud, cloud_normals, 10, 0.05, "normals");
    viewer->spin();
}

//
void calPointNormal(const pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::Normal &normal) {
    // calculate normal for a point cloud by PCA

    // from PCL format to PointCloud format
    PointCloud::Ptr temp_cloud_ptr(new PointCloud());
    for (auto &pti : cloud) {
        Point::Ptr temp_point_ptr(new Point(Vec3(pti.x, pti.y, pti.z)));
        temp_cloud_ptr->InsertPoint(temp_point_ptr);
    }

    // PCASolver
    PCASolver::Ptr temp_pca_solver_ptr(new PCASolver(temp_cloud_ptr));
    auto minEigenVector = temp_pca_solver_ptr->GetEigenVectors().col(2);
    auto allEigenValues = temp_pca_solver_ptr->GetEigenValues();

    normal.normal_x = minEigenVector.x();
    normal.normal_y = minEigenVector.y();
    normal.normal_z = minEigenVector.z();
    normal.curvature = static_cast<float>(allEigenValues[2] / (allEigenValues[0] + allEigenValues[1] + allEigenValues[2]));
}
