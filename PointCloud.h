// ----------------------------------------------------------------------------
// -                        PCD RW                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 JD
// ----------------------------------------------------------------------------

#pragma once

#include <Eigen/Core>
#include <memory>
#include <tuple>
#include <vector>

#include "Geometry3D.h"

namespace pcd
{
        namespace geometry
        {

                class Image;
                class RGBDImage;
                class TriangleMesh;
                class VoxelGrid;

                /// \class PointCloud
                ///
                /// \brief A point cloud consists of point coordinates, and optionally point
                /// colors and point normals.
                class PCDIO_EXPORTS PointCloud : public Geometry3D
                {
                public:
                        /// \brief Default Constructor.
                        PointCloud() : Geometry3D(Geometry::GeometryType::PointCloud) {}
                        /// \brief Parameterized Constructor.
                        ///
                        /// \param points Points coordinates.
                        PointCloud(const std::vector<Eigen::Vector3d> &points)
                            : Geometry3D(Geometry::GeometryType::PointCloud), points_(points) {}
                        ~PointCloud() override {}

                public:
                        PointCloud &Clear() override;
                        bool IsEmpty() const override;
                        Eigen::Vector3d GetMinBound() const override;
                        Eigen::Vector3d GetMaxBound() const override;
                        Eigen::Vector3d GetCenter() const override;
                        PointCloud &Transform(const Eigen::Matrix4d &transformation) override;
                        PointCloud &Translate(const Eigen::Vector3d &translation,
                                              bool relative = true) override;
                        PointCloud &Scale(const double scale,
                                          const Eigen::Vector3d &center) override;
                        PointCloud &Rotate(const Eigen::Matrix3d &R,
                                           const Eigen::Vector3d &center) override;

                        PointCloud &operator+=(const PointCloud &cloud);

                        /// Returns 'true' if the point cloud contains points.
                        bool HasPoints() const { return points_.size() > 0; }

                        /// Returns `true` if the point cloud contains point normals.
                        bool HasIntensitys() const
                        {
                                return points_.size() > 0 && intensitys_.size() == intensitys_.size();
                        }

                        /// Returns `true` if the point cloud contains point normals.
                        bool HasNormals() const
                        {
                                return points_.size() > 0 && normals_.size() == points_.size();
                        }

                        /// Returns `true` if the point cloud contains point colors.
                        bool HasColors() const
                        {
                                return points_.size() > 0 && colors_.size() == points_.size();
                        }

                        /// Returns 'true' if the point cloud contains per-point covariance matrix.
                        bool HasCovariances() const
                        {
                                return !points_.empty() && covariances_.size() == points_.size();
                        }

                        /// Normalize point normals to length 1.
                        PointCloud &NormalizeNormals()
                        {
                                for (size_t i = 0; i < normals_.size(); i++)
                                {
                                        normals_[i].normalize();
                                }
                                return *this;
                        }

                        /// Assigns each point in the PointCloud the same color.
                        ///
                        /// \param color  RGB colors of points.
                        PointCloud &PaintUniformColor(const Eigen::Vector3d &color)
                        {
                                ResizeAndPaintUniformColor(colors_, points_.size(), color);
                                return *this;
                        }

                        /// \brief Removes all points from the point cloud that have a nan entry, or
                        /// infinite entries. It also removes the corresponding attributes
                        /// associated with the non-finite point such as normals, covariances and
                        /// color entries. It doesn't re-computes these attributes after removing
                        /// non-finite points.
                        ///
                        /// \param remove_nan Remove NaN values from the PointCloud.
                        /// \param remove_infinite Remove infinite values from the PointCloud.
                        PointCloud &RemoveNonFinitePoints(bool remove_nan = true,
                                                          bool remove_infinite = true);

                        /// \brief Selects points from \p input pointcloud, with indices in \p
                        /// indices, and returns a new point-cloud with selected points.
                        ///
                        /// \param indices Indices of points to be selected.
                        /// \param invert Set to `True` to invert the selection of indices.
                        std::shared_ptr<PointCloud> SelectByIndex(
                            const std::vector<size_t> &indices, bool invert = false) const;

                public:
                        /// Points coordinates.
                        std::vector<Eigen::Vector3d> points_;
                        /// Points intensity
                        std::vector<float> intensitys_;
                        /// Points normals.
                        std::vector<Eigen::Vector3d> normals_;
                        /// RGB colors of points.
                        std::vector<Eigen::Vector3d> colors_;
                        /// Covariance Matrix for each point
                        std::vector<Eigen::Matrix3d> covariances_;
                };

        } // namespace geometry
} // namespace open3d
