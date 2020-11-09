#include "voxelgridfilter.h"

VoxelGridFilter::VoxelGridFilter(const PointCloud::Ptr &origin_pointcloud_ptr,
                                 Vec3 voxel_grid_size,
                                 VoxelGridFilter::ALGO algorithm)
                                 : origin_pointcloud_ptr_(origin_pointcloud_ptr),
                                   voxel_grid_size_(std::move(voxel_grid_size)),
                                   algorithm_(algorithm) {
    // initialization
    std::cout << "\n[ VoxelGridFilter Initializing... ]" << std::endl;

    PointCloud::Ptr ptr(new PointCloud());
    filtered_pointcloud_ptr_ = ptr;

    // voxel grid filtering
    voxel_grid_filter();

    std::cout << "[ VoxelGridFilter Initialization Finished. ]" << std::endl;
}

VecVector3d VoxelGridFilter::minmaxVertex(const PointCloud::Ptr &point_cloud_ptr) {
    // each point
    VecVector3d all_points_coor;
    auto all_points = point_cloud_ptr->GetAllPoints();
    for (auto &pt : all_points) {
        auto pt_coor = pt->GetPos();
        all_points_coor.push_back(pt_coor);
    }
    // x, y, z
    VecVector3d minmaxVertexVec(2);
    for (int i = 0; i < 3; ++i) {
        std::sort(all_points_coor.begin(), all_points_coor.end(),
                  [i](const Vec3 &coor1, const Vec3 &coor2) -> bool {
                      return coor1[i] < coor2[i];
                  });
        minmaxVertexVec[0].row(i) = all_points_coor[0].row(i);
        minmaxVertexVec[1].row(i) = all_points_coor.back().row(i);
    }

    return minmaxVertexVec;
}

void VoxelGridFilter::filter_algorithm(const std::map<long long, int> &index_and_id) {
    // filter algorithm: CENTROID or RANDOM
    if (algorithm_ == ALGO::CENTROID) {
        auto iter0 = index_and_id.begin();
        for (auto iter = index_and_id.begin(); iter != index_and_id.end(); ++iter) {
            int count = 1;
            Vec3 centroid = Vec3::Zero(3, 1);
            centroid += origin_pointcloud_ptr_->GetAllPoints()[iter->second]->GetPos();

            // iter0 = iter;
            while (++iter0 != index_and_id.end() && iter0->first == iter->first) {
                ++iter;
                ++count;
                centroid += origin_pointcloud_ptr_->GetAllPoints()[iter->second]->GetPos();
            }
            centroid /= count;

            Point::Ptr pt_ptr(new Point(centroid));
            filtered_pointcloud_ptr_->InsertPoint(pt_ptr);
        }
    } else if (algorithm_ == ALGO::RANDOM) {
        srand(time(nullptr));

        auto iter0 = index_and_id.begin();
        for (auto iter = index_and_id.begin(); iter != index_and_id.end(); ++iter) {
            std::vector<int> sameId;
            sameId.push_back(iter->second);

            // iter0 = iter;
            while (++iter0 != index_and_id.end() && iter0->first == iter->first) {
                ++iter;
                sameId.push_back(iter->second);
            }

            int randomId = sameId[rand() % sameId.size()];
            Vec3 randPos = origin_pointcloud_ptr_->GetAllPoints()[randomId]->GetPos();

            Point::Ptr pt_ptr(new Point(randPos));
            filtered_pointcloud_ptr_->InsertPoint(pt_ptr);
        }
    }
}

void VoxelGridFilter::voxel_grid_filter() {
    // voxel dimension
    auto minmaxVertexCoor = minmaxVertex(origin_pointcloud_ptr_);
    Vec3 minVertex = minmaxVertexCoor[0];
    Vec3 maxVertex = minmaxVertexCoor[1];

    long long nx, ny, nz;
    nx = static_cast<long long>((maxVertex.x()-minVertex.x()) / voxel_grid_size_.x());
    ny = static_cast<long long>((maxVertex.y()-minVertex.y()) / voxel_grid_size_.y());
    nz = static_cast<long long>((maxVertex.z()-minVertex.z()) / voxel_grid_size_.z());
    // check data type overflow
    if (nx < 0 || ny < 0 || nz < 0 || (nx*ny*nz) > static_cast<long long>(std::numeric_limits<int>::max())) {
        std::cerr << "Error: data type overflow. " << std::endl;
        return;
    }

    // each point's index and id
    // std::map -> automatic sorting
    std::map<long long, int> index_and_id;
    auto all_points = origin_pointcloud_ptr_->GetAllPoints();
    auto all_points_num = all_points.size();
    for (int i = 0; i < all_points_num; ++i) {
        auto pos = all_points[i]->GetPos();

        long long hx = std::floor(1.0 * (pos.x()-minVertex.x()) / voxel_grid_size_.x());
        long long hy = std::floor(1.0 * (pos.y()-minVertex.y()) / voxel_grid_size_.y());
        long long hz = std::floor(1.0 * (pos.z()-minVertex.z()) / voxel_grid_size_.z());
        long long index = hx + hy*nx + hz*nx*ny;

        index_and_id.insert(std::make_pair(index, i));
    }

    // filtering
    filter_algorithm(index_and_id);
}
