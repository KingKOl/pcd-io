// ----------------------------------------------------------------------------
// -                        PCD RW                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 JD
// ----------------------------------------------------------------------------
#pragma once

#include <functional>
#include "PointCloud.h"

namespace pcd
{
    namespace io
    {
        /// \struct WritePointCloudOption
        /// \brief Optional parameters to WritePointCloud
        struct WritePointCloudOption
        {
            enum class IsAscii : bool
            {
                Binary = false,
                Ascii = true
            };
            enum class Compressed : bool
            {
                Uncompressed = false,
                Compressed = true
            };
            WritePointCloudOption(
                // Attention: when you update the defaults, update the docstrings in
                // pybind/io/class_io.cpp
                IsAscii write_ascii = IsAscii::Binary,
                Compressed compressed = Compressed::Uncompressed,
                bool print_progress = false,
                std::function<bool(double)> update_progress = {})
                : write_ascii(write_ascii),
                  compressed(compressed),
                  print_progress(print_progress),
                  update_progress(update_progress){};
            // for compatibility
            WritePointCloudOption(bool write_ascii,
                                  bool compressed = false,
                                  bool print_progress = false,
                                  std::function<bool(double)> update_progress = {})
                : write_ascii(IsAscii(write_ascii)),
                  compressed(Compressed(compressed)),
                  print_progress(print_progress),
                  update_progress(update_progress){};
            WritePointCloudOption(std::function<bool(double)> up)
                : WritePointCloudOption()
            {
                update_progress = up;
            };
            /// Whether to save in Ascii or Binary.  Some savers are capable of doing
            /// either, other ignore this.
            IsAscii write_ascii;
            /// Whether to save Compressed or Uncompressed.  Currently, only PCD is
            /// capable of compressing, and only if using IsAscii::Binary, all other
            /// formats ignore this.
            Compressed compressed;
            /// Print progress to stdout about loading progress.  Also see
            /// \p update_progress if you want to have your own progress indicators or
            /// to be able to cancel loading.
            bool print_progress;
            /// Callback to invoke as reading is progressing, parameter is percentage
            /// completion (0.-100.) return true indicates to continue loading, false
            /// means to try to stop loading and cleanup
            std::function<bool(double)> update_progress;
        };

        /// \struct ReadPointCloudOption
        /// \brief Optional parameters to ReadPointCloud
        struct ReadPointCloudOption
        {
            ReadPointCloudOption(
                // Attention: when you update the defaults, update the docstrings in
                // pybind/io/class_io.cpp
                std::string format = "auto",
                bool remove_nan_points = false,
                bool remove_infinite_points = false,
                bool print_progress = false,
                std::function<bool(double)> update_progress = {})
                : format(format),
                  remove_nan_points(remove_nan_points),
                  remove_infinite_points(remove_infinite_points),
                  print_progress(print_progress),
                  update_progress(update_progress){};
            ReadPointCloudOption(std::function<bool(double)> up)
                : ReadPointCloudOption()
            {
                update_progress = up;
            };
            /// Specifies what format the contents of the file are (and what loader to
            /// use), default "auto" means to go off of file extension.
            std::string format;
            /// Whether to remove all points that have nan
            bool remove_nan_points;
            /// Whether to remove all points that have +-inf
            bool remove_infinite_points;
            /// Print progress to stdout about loading progress.
            /// Also see \p update_progress if you want to have your own progress
            /// indicators or to be able to cancel loading.
            bool print_progress;
            /// Callback to invoke as reading is progressing, parameter is percentage
            /// completion (0.-100.) return true indicates to continue loading, false
            /// means to try to stop loading and cleanup
            std::function<bool(double)> update_progress;
        };

        PCDIO_EXPORTS bool ReadPointCloudFromPCD(const std::string &filename,
                                                 geometry::PointCloud &pointcloud);

        PCDIO_EXPORTS bool WritePointCloudToPCD(const std::string &filename,
                                                const geometry::PointCloud &pointcloud,
                                                const WritePointCloudOption &params);
    }
}