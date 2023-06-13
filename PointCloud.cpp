// ----------------------------------------------------------------------------
// -                        PCD RW                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 JD
// ----------------------------------------------------------------------------

#include "PointCloud.h"

#include <Eigen/Dense>
#include <algorithm>
#include <numeric>

namespace pcd
{
    namespace geometry
    {

        PointCloud &PointCloud::Clear()
        {
            points_.clear();
            intensitys_.clear();
            normals_.clear();
            colors_.clear();
            covariances_.clear();
            return *this;
        }

        bool PointCloud::IsEmpty() const { return !HasPoints(); }

        Eigen::Vector3d PointCloud::GetMinBound() const
        {
            return ComputeMinBound(points_);
        }

        Eigen::Vector3d PointCloud::GetMaxBound() const
        {
            return ComputeMaxBound(points_);
        }

        Eigen::Vector3d PointCloud::GetCenter() const { return ComputeCenter(points_); }

        PointCloud &PointCloud::Transform(const Eigen::Matrix4d &transformation)
        {
            TransformPoints(transformation, points_);
            TransformNormals(transformation, normals_);
            TransformCovariances(transformation, covariances_);
            return *this;
        }

        PointCloud &PointCloud::Translate(const Eigen::Vector3d &translation,
                                          bool relative)
        {
            TranslatePoints(translation, points_, relative);
            return *this;
        }

        PointCloud &PointCloud::Scale(const double scale,
                                      const Eigen::Vector3d &center)
        {
            ScalePoints(scale, points_, center);
            return *this;
        }

        PointCloud &PointCloud::Rotate(const Eigen::Matrix3d &R,
                                       const Eigen::Vector3d &center)
        {
            RotatePoints(R, points_, center);
            RotateNormals(R, normals_);
            RotateCovariances(R, covariances_);
            return *this;
        }

        PointCloud &PointCloud::operator+=(const PointCloud &cloud)
        {
            // We do not use std::vector::insert to combine std::vector because it will
            // crash if the pointcloud is added to itself.
            if (cloud.IsEmpty())
                return (*this);
            size_t old_vert_num = points_.size();
            size_t add_vert_num = cloud.points_.size();
            size_t new_vert_num = old_vert_num + add_vert_num;
            if ((!HasPoints() || HasIntensitys()) && cloud.HasIntensitys())
            {
                intensitys_.resize(new_vert_num);
                for (size_t i = 0; i < add_vert_num; i++)
                    intensitys_[old_vert_num + i] = cloud.intensitys_[i];
            }
            else
            {
                intensitys_.clear();
            }
            if ((!HasPoints() || HasNormals()) && cloud.HasNormals())
            {
                normals_.resize(new_vert_num);
                for (size_t i = 0; i < add_vert_num; i++)
                    normals_[old_vert_num + i] = cloud.normals_[i];
            }
            else
            {
                normals_.clear();
            }
            if ((!HasPoints() || HasColors()) && cloud.HasColors())
            {
                colors_.resize(new_vert_num);
                for (size_t i = 0; i < add_vert_num; i++)
                    colors_[old_vert_num + i] = cloud.colors_[i];
            }
            else
            {
                colors_.clear();
            }
            if ((!HasPoints() || HasCovariances()) && cloud.HasCovariances())
            {
                covariances_.resize(new_vert_num);
                for (size_t i = 0; i < add_vert_num; i++)
                    covariances_[old_vert_num + i] = cloud.covariances_[i];
            }
            else
            {
                covariances_.clear();
            }
            points_.resize(new_vert_num);
            for (size_t i = 0; i < add_vert_num; i++)
                points_[old_vert_num + i] = cloud.points_[i];
            return (*this);
        }

        PointCloud &PointCloud::RemoveNonFinitePoints(bool remove_nan,
                                                      bool remove_infinite)
        {
            bool has_intensity = HasIntensitys();
            bool has_normal = HasNormals();
            bool has_color = HasColors();
            bool has_covariance = HasCovariances();
            size_t old_point_num = points_.size();
            size_t k = 0; // new index
            for (size_t i = 0; i < old_point_num; i++)
            { // old index
                bool is_nan = remove_nan &&
                              (std::isnan(points_[i](0)) || std::isnan(points_[i](1)) ||
                               std::isnan(points_[i](2)));
                bool is_infinite = remove_infinite && (std::isinf(points_[i](0)) ||
                                                       std::isinf(points_[i](1)) ||
                                                       std::isinf(points_[i](2)));
                if (!is_nan && !is_infinite)
                {
                    points_[k] = points_[i];
                    if (has_intensity)
                        intensitys_[k] = intensitys_[i];
                    if (has_normal)
                        normals_[k] = normals_[i];
                    if (has_color)
                        colors_[k] = colors_[i];
                    if (has_covariance)
                        covariances_[k] = covariances_[i];
                    k++;
                }
            }

            points_.resize(k);
            if (has_intensity)
                intensitys_.resize(k);
            if (has_normal)
                normals_.resize(k);
            if (has_color)
                colors_.resize(k);
            if (has_covariance)
                covariances_.resize(k);

            fprintf(stderr,
                    "[RemoveNonFinitePoints] %d nan points have been removed.\n",
                    (int)(old_point_num - k));

            return *this;
        }

        std::shared_ptr<PointCloud> PointCloud::SelectByIndex(
            const std::vector<size_t> &indices, bool invert /* = false */) const
        {
            auto output = std::make_shared<PointCloud>();
            bool has_intensity = HasIntensitys();
            bool has_normals = HasNormals();
            bool has_colors = HasColors();
            bool has_covariance = HasCovariances();

            std::vector<bool> mask = std::vector<bool>(points_.size(), invert);
            for (size_t i : indices)
            {
                mask[i] = !invert;
            }

            for (size_t i = 0; i < points_.size(); i++)
            {
                if (mask[i])
                {
                    output->points_.push_back(points_[i]);
                    if (has_intensity)
                        output->intensitys_.push_back(intensitys_[i]);
                    if (has_normals)
                        output->normals_.push_back(normals_[i]);
                    if (has_colors)
                        output->colors_.push_back(colors_[i]);
                    if (has_covariance)
                        output->covariances_.push_back(covariances_[i]);
                }
            }

            fprintf(stderr,
                    "Pointcloud down sampled from %d points to %d points.\n",
                    (int)points_.size(), (int)output->points_.size());

            return output;
        }
    } // namespace geometry
} // namespace open3d
