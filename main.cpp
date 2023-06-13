#include "PointCloudIO.h"

int main(int argc, char **argv)
{
    auto cloud_ptr = std::make_shared<pcd::geometry::PointCloud>();
    pcd::io::ReadPointCloudFromPCD("/mnt/d/010734.pcd", *cloud_ptr);
    fprintf(stderr, "Point count: %ld\n", cloud_ptr->points_.size());
    for (size_t i = 0; i < cloud_ptr->points_.size(); i++)
    {
        /* code */
        cloud_ptr->intensitys_.at(i) *= 255;
        Eigen::Vector3d &xyz_ = cloud_ptr->points_.at(i);
        fprintf(stderr, "Point %ld: [x:%f y:%f z:%f i:%f]\n", i, xyz_[0], xyz_[1], xyz_[1], cloud_ptr->intensitys_.at(i));
    }

    pcd::io::WritePointCloudOption option_(false, true);
    pcd::io::WritePointCloudToPCD("/mnt/d/010734-1.pcd", *cloud_ptr, option_);

    return 0;
}