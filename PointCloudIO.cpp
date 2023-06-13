// ----------------------------------------------------------------------------
// -                        PCD RW                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2023 JD
// ----------------------------------------------------------------------------
#include "PointCloudIO.h"

#include <cstdint>
#include <cstdio>
#include <sstream>
#include <vector>
#include <string.h>

#include "LZF.h"

#define DEFAULT_IO_BUFFER_SIZE 1024

namespace pcd
{
    namespace
    {
        using namespace io;

        std::vector<std::string> SplitString(const std::string &str,
                                             const std::string &delimiters /* = " "*/,
                                             bool trim_empty_str = true)
        {
            std::vector<std::string> tokens;
            std::string::size_type pos = 0, new_pos = 0, last_pos = 0;
            while (pos != std::string::npos)
            {
                pos = str.find_first_of(delimiters, last_pos);
                new_pos = (pos == std::string::npos ? str.length() : pos);
                if (new_pos != last_pos || !trim_empty_str)
                {
                    tokens.push_back(str.substr(last_pos, new_pos - last_pos));
                }
                last_pos = new_pos + 1;
            }
            return tokens;
        }

        Eigen::Vector3d ColorToDouble(uint8_t r, uint8_t g, uint8_t b)
        {
            return Eigen::Vector3d(r, g, b) / 255.0;
        }

        typedef Eigen::Matrix<uint8_t, 3, 1> Vector3uint8;
        Vector3uint8 ColorToUint8(const Eigen::Vector3d &color)
        {
            Vector3uint8 rgb;
            for (int i = 0; i < 3; ++i)
            {
                rgb[i] = uint8_t(
                    std::round(std::min(1., std::max(0., color(i))) * 255.));
            }
            return rgb;
        }

        enum PCDDataType
        {
            PCD_DATA_ASCII = 0,
            PCD_DATA_BINARY = 1,
            PCD_DATA_BINARY_COMPRESSED = 2
        };

        struct PCLPointField
        {
        public:
            std::string name;
            int size;
            char type;
            int count;
            // helper variable
            int count_offset;
            int offset;
        };

        struct PCDHeader
        {
        public:
            std::string version;
            std::vector<PCLPointField> fields;
            int width;
            int height;
            int points;
            PCDDataType datatype;
            std::string viewpoint;
            // helper variables
            int elementnum;
            int pointsize;
            bool has_points;
            bool has_intensitys;
            bool has_normals;
            bool has_colors;
        };

        bool CheckHeader(PCDHeader &header)
        {
            if (header.points <= 0 || header.pointsize <= 0)
            {
                fprintf(stderr, "[CheckHeader] PCD has no data.\n");
                return false;
            }
            if (header.fields.size() == 0 || header.pointsize <= 0)
            {
                fprintf(stderr, "[CheckHeader] PCD has no fields.\n");
                return false;
            }
            header.has_points = false;
            header.has_intensitys = false;
            header.has_normals = false;
            header.has_colors = false;
            bool has_x = false;
            bool has_y = false;
            bool has_z = false;
            bool has_normal_x = false;
            bool has_normal_y = false;
            bool has_normal_z = false;
            bool has_rgb = false;
            bool has_rgba = false;
            bool has_intensity = false;
            for (const auto &field : header.fields)
            {
                if (field.name == "x")
                {
                    has_x = true;
                }
                else if (field.name == "y")
                {
                    has_y = true;
                }
                else if (field.name == "z")
                {
                    has_z = true;
                }
                else if (field.name == "intensity")
                {
                    has_intensity = true;
                }
                else if (field.name == "normal_x")
                {
                    has_normal_x = true;
                }
                else if (field.name == "normal_y")
                {
                    has_normal_y = true;
                }
                else if (field.name == "normal_z")
                {
                    has_normal_z = true;
                }
                else if (field.name == "rgb")
                {
                    has_rgb = true;
                }
                else if (field.name == "rgba")
                {
                    has_rgba = true;
                }
            }
            header.has_points = (has_x && has_y && has_z);
            header.has_intensitys = has_intensity;
            header.has_normals = (has_normal_x && has_normal_y && has_normal_z);
            header.has_colors = (has_rgb || has_rgba);
            if (!header.has_points)
            {
                fprintf(stderr, "[CheckHeader] Fields for point data are not complete.\n");
                return false;
            }
            return true;
        }

        bool ReadPCDHeader(FILE *file, PCDHeader &header)
        {
            char line_buffer[DEFAULT_IO_BUFFER_SIZE];
            size_t specified_channel_count = 0;

            while (fgets(line_buffer, DEFAULT_IO_BUFFER_SIZE, file))
            {
                std::string line(line_buffer);
                if (line == "")
                {
                    continue;
                }
                std::vector<std::string> st = SplitString(line, "\t\r\n ");
                std::stringstream sstream(line);
                sstream.imbue(std::locale::classic());
                std::string line_type;
                sstream >> line_type;
                if (line_type.substr(0, 1) == "#")
                {
                }
                else if (line_type.substr(0, 7) == "VERSION")
                {
                    if (st.size() >= 2)
                    {
                        header.version = st[1];
                    }
                }
                else if (line_type.substr(0, 6) == "FIELDS" ||
                         line_type.substr(0, 7) == "COLUMNS")
                {
                    specified_channel_count = st.size() - 1;
                    if (specified_channel_count == 0)
                    {
                        fprintf(stderr, "[ReadPCDHeader] Bad PCD file format.\n");
                        return false;
                    }
                    header.fields.resize(specified_channel_count);
                    int count_offset = 0, offset = 0;
                    for (size_t i = 0; i < specified_channel_count;
                         i++, count_offset += 1, offset += 4)
                    {
                        header.fields[i].name = st[i + 1];
                        header.fields[i].size = 4;
                        header.fields[i].type = 'F';
                        header.fields[i].count = 1;
                        header.fields[i].count_offset = count_offset;
                        header.fields[i].offset = offset;
                    }
                    header.elementnum = count_offset;
                    header.pointsize = offset;
                }
                else if (line_type.substr(0, 4) == "SIZE")
                {
                    if (specified_channel_count != st.size() - 1)
                    {
                        fprintf(stderr, "[ReadPCDHeader] Bad PCD file format.\n");
                        return false;
                    }
                    int offset = 0, col_type = 0;
                    for (size_t i = 0; i < specified_channel_count;
                         i++, offset += col_type)
                    {
                        sstream >> col_type;
                        header.fields[i].size = col_type;
                        header.fields[i].offset = offset;
                    }
                    header.pointsize = offset;
                }
                else if (line_type.substr(0, 4) == "TYPE")
                {
                    if (specified_channel_count != st.size() - 1)
                    {
                        fprintf(stderr, "[ReadPCDHeader] Bad PCD file format.\n");
                        return false;
                    }
                    for (size_t i = 0; i < specified_channel_count; i++)
                    {
                        header.fields[i].type = st[i + 1].c_str()[0];
                    }
                }
                else if (line_type.substr(0, 5) == "COUNT")
                {
                    if (specified_channel_count != st.size() - 1)
                    {
                        fprintf(stderr, "[ReadPCDHeader] Bad PCD file format.\n");
                        return false;
                    }
                    int count_offset = 0, offset = 0, col_count = 0;
                    for (size_t i = 0; i < specified_channel_count; i++)
                    {
                        sstream >> col_count;
                        header.fields[i].count = col_count;
                        header.fields[i].count_offset = count_offset;
                        header.fields[i].offset = offset;
                        count_offset += col_count;
                        offset += col_count * header.fields[i].size;
                    }
                    header.elementnum = count_offset;
                    header.pointsize = offset;
                }
                else if (line_type.substr(0, 5) == "WIDTH")
                {
                    sstream >> header.width;
                }
                else if (line_type.substr(0, 6) == "HEIGHT")
                {
                    sstream >> header.height;
                    header.points = header.width * header.height;
                }
                else if (line_type.substr(0, 9) == "VIEWPOINT")
                {
                    if (st.size() >= 2)
                    {
                        header.viewpoint = st[1];
                    }
                }
                else if (line_type.substr(0, 6) == "POINTS")
                {
                    sstream >> header.points;
                }
                else if (line_type.substr(0, 4) == "DATA")
                {
                    header.datatype = PCD_DATA_ASCII;
                    if (st.size() >= 2)
                    {
                        if (st[1].substr(0, 17) == "binary_compressed")
                        {
                            header.datatype = PCD_DATA_BINARY_COMPRESSED;
                        }
                        else if (st[1].substr(0, 6) == "binary")
                        {
                            header.datatype = PCD_DATA_BINARY;
                        }
                    }
                    break;
                }
            }
            if (!CheckHeader(header))
            {
                return false;
            }
            return true;
        }

        double UnpackBinaryPCDElement(const char *data_ptr,
                                      const char type,
                                      const int size)
        {
            if (type == 'I')
            {
                if (size == 1)
                {
                    std::int8_t data;
                    memcpy(&data, data_ptr, sizeof(data));
                    return (double)data;
                }
                else if (size == 2)
                {
                    std::int16_t data;
                    memcpy(&data, data_ptr, sizeof(data));
                    return (double)data;
                }
                else if (size == 4)
                {
                    std::int32_t data;
                    memcpy(&data, data_ptr, sizeof(data));
                    return (double)data;
                }
                else
                {
                    return 0.0;
                }
            }
            else if (type == 'U')
            {
                if (size == 1)
                {
                    std::uint8_t data;
                    memcpy(&data, data_ptr, sizeof(data));
                    return (double)data;
                }
                else if (size == 2)
                {
                    std::uint16_t data;
                    memcpy(&data, data_ptr, sizeof(data));
                    return (double)data;
                }
                else if (size == 4)
                {
                    std::uint32_t data;
                    memcpy(&data, data_ptr, sizeof(data));
                    return (double)data;
                }
                else
                {
                    return 0.0;
                }
            }
            else if (type == 'F')
            {
                if (size == 4)
                {
                    float data;
                    memcpy(&data, data_ptr, sizeof(data));
                    return (double)data;
                }
                else
                {
                    return 0.0;
                }
            }
            return 0.0;
        }

        Eigen::Vector3d UnpackBinaryPCDColor(const char *data_ptr,
                                             const char type,
                                             const int size)
        {
            if (size == 4)
            {
                std::uint8_t data[4];
                memcpy(data, data_ptr, 4);
                // color data is packed in BGR order.
                return ColorToDouble(data[2], data[1], data[0]);
            }
            else
            {
                return Eigen::Vector3d::Zero();
            }
        }

        double UnpackASCIIPCDElement(const char *data_ptr,
                                     const char type,
                                     const int size)
        {
            char *end;
            if (type == 'I')
            {
                return (double)std::strtol(data_ptr, &end, 0);
            }
            else if (type == 'U')
            {
                return (double)std::strtoul(data_ptr, &end, 0);
            }
            else if (type == 'F')
            {
                return std::strtod(data_ptr, &end);
            }
            return 0.0;
        }

        Eigen::Vector3d UnpackASCIIPCDColor(const char *data_ptr,
                                            const char type,
                                            const int size)
        {
            if (size == 4)
            {
                std::uint8_t data[4] = {0, 0, 0, 0};
                char *end;
                if (type == 'I')
                {
                    std::int32_t value = std::strtol(data_ptr, &end, 0);
                    memcpy(data, &value, 4);
                }
                else if (type == 'U')
                {
                    std::uint32_t value = std::strtoul(data_ptr, &end, 0);
                    memcpy(data, &value, 4);
                }
                else if (type == 'F')
                {
                    float value = std::strtof(data_ptr, &end);
                    memcpy(data, &value, 4);
                }
                return ColorToDouble(data[2], data[1], data[0]);
            }
            else
            {
                return Eigen::Vector3d::Zero();
            }
        }

        bool ReadPCDData(FILE *file,
                         const PCDHeader &header,
                         geometry::PointCloud &pointcloud)
        {
            // The header should have been checked
            if (header.has_points)
            {
                pointcloud.points_.resize(header.points);
            }
            else
            {
                fprintf(stderr, "[ReadPCDData] Fields for point data are not complete.\n");
                return false;
            }
            if (header.has_intensitys)
            {
                pointcloud.intensitys_.resize(header.points);
            }
            if (header.has_normals)
            {
                pointcloud.normals_.resize(header.points);
            }
            if (header.has_colors)
            {
                pointcloud.colors_.resize(header.points);
            }

            if (header.datatype == PCD_DATA_ASCII)
            {
                char line_buffer[DEFAULT_IO_BUFFER_SIZE];
                int idx = 0;
                while (fgets(line_buffer, DEFAULT_IO_BUFFER_SIZE, file) &&
                       idx < header.points)
                {
                    std::string line(line_buffer);
                    std::vector<std::string> strs = SplitString(line, "\t\r\n ");
                    if ((int)strs.size() < header.elementnum)
                    {
                        continue;
                    }
                    for (size_t i = 0; i < header.fields.size(); i++)
                    {
                        const auto &field = header.fields[i];
                        if (field.name == "x")
                        {
                            pointcloud.points_[idx](0) = UnpackASCIIPCDElement(
                                strs[field.count_offset].c_str(), field.type,
                                field.size);
                        }
                        else if (field.name == "y")
                        {
                            pointcloud.points_[idx](1) = UnpackASCIIPCDElement(
                                strs[field.count_offset].c_str(), field.type,
                                field.size);
                        }
                        else if (field.name == "z")
                        {
                            pointcloud.points_[idx](2) = UnpackASCIIPCDElement(
                                strs[field.count_offset].c_str(), field.type,
                                field.size);
                        }
                        else if (field.name == "intensity")
                        {
                            pointcloud.intensitys_[idx] = UnpackASCIIPCDElement(
                                strs[field.count_offset].c_str(), field.type,
                                field.size);
                        }
                        else if (field.name == "normal_x")
                        {
                            pointcloud.normals_[idx](0) = UnpackASCIIPCDElement(
                                strs[field.count_offset].c_str(), field.type,
                                field.size);
                        }
                        else if (field.name == "normal_y")
                        {
                            pointcloud.normals_[idx](1) = UnpackASCIIPCDElement(
                                strs[field.count_offset].c_str(), field.type,
                                field.size);
                        }
                        else if (field.name == "normal_z")
                        {
                            pointcloud.normals_[idx](2) = UnpackASCIIPCDElement(
                                strs[field.count_offset].c_str(), field.type,
                                field.size);
                        }
                        else if (field.name == "rgb" || field.name == "rgba")
                        {
                            pointcloud.colors_[idx] = UnpackASCIIPCDColor(
                                strs[field.count_offset].c_str(), field.type,
                                field.size);
                        }
                    }
                    idx++;
                }
            }
            else if (header.datatype == PCD_DATA_BINARY)
            {
                std::unique_ptr<char[]> buffer(new char[header.pointsize]);
                for (int i = 0; i < header.points; i++)
                {
                    if (fread(buffer.get(), header.pointsize, 1, file) != 1)
                    {
                        fprintf(stderr, "[ReadPCDData] Failed to read data record.\n");
                        pointcloud.Clear();
                        return false;
                    }
                    for (const auto &field : header.fields)
                    {
                        if (field.name == "x")
                        {
                            pointcloud.points_[i](0) =
                                UnpackBinaryPCDElement(buffer.get() + field.offset,
                                                       field.type, field.size);
                        }
                        else if (field.name == "y")
                        {
                            pointcloud.points_[i](1) =
                                UnpackBinaryPCDElement(buffer.get() + field.offset,
                                                       field.type, field.size);
                        }
                        else if (field.name == "z")
                        {
                            pointcloud.points_[i](2) =
                                UnpackBinaryPCDElement(buffer.get() + field.offset,
                                                       field.type, field.size);
                        }
                        else if (field.name == "intensity")
                        {
                            pointcloud.intensitys_[i] =
                                UnpackBinaryPCDElement(buffer.get() + field.offset,
                                                       field.type, field.size);
                        }
                        else if (field.name == "normal_x")
                        {
                            pointcloud.normals_[i](0) =
                                UnpackBinaryPCDElement(buffer.get() + field.offset,
                                                       field.type, field.size);
                        }
                        else if (field.name == "normal_y")
                        {
                            pointcloud.normals_[i](1) =
                                UnpackBinaryPCDElement(buffer.get() + field.offset,
                                                       field.type, field.size);
                        }
                        else if (field.name == "normal_z")
                        {
                            pointcloud.normals_[i](2) =
                                UnpackBinaryPCDElement(buffer.get() + field.offset,
                                                       field.type, field.size);
                        }
                        else if (field.name == "rgb" || field.name == "rgba")
                        {
                            pointcloud.colors_[i] =
                                UnpackBinaryPCDColor(buffer.get() + field.offset,
                                                     field.type, field.size);
                        }
                    }
                }
            }
            else if (header.datatype == PCD_DATA_BINARY_COMPRESSED)
            {
                double reporter_total = 100.0;
                std::uint32_t compressed_size;
                std::uint32_t uncompressed_size;
                if (fread(&compressed_size, sizeof(compressed_size), 1, file) != 1)
                {
                    fprintf(stderr, "[ReadPCDData] Failed to read data record.\n");
                    pointcloud.Clear();
                    return false;
                }
                if (fread(&uncompressed_size, sizeof(uncompressed_size), 1, file) !=
                    1)
                {
                    fprintf(stderr, "[ReadPCDData] Failed to read data record.\n");
                    pointcloud.Clear();
                    return false;
                }
                fprintf(stderr, "PCD data with %d compressed size, and %d uncompressed size.\n",
                        compressed_size, uncompressed_size);
                std::unique_ptr<char[]> buffer_compressed(new char[compressed_size]);
                if (fread(buffer_compressed.get(), 1, compressed_size, file) !=
                    compressed_size)
                {
                    fprintf(stderr, "[ReadPCDData] Failed to read data record.\n");
                    pointcloud.Clear();
                    return false;
                }
                std::unique_ptr<char[]> buffer(new char[uncompressed_size]);
                if (lzfDecompress(buffer_compressed.get(),
                                  (unsigned int)compressed_size, buffer.get(),
                                  (unsigned int)uncompressed_size) !=
                    uncompressed_size)
                {
                    fprintf(stderr, "[ReadPCDData] Uncompression failed.\n");
                    pointcloud.Clear();
                    return false;
                }
                for (const auto &field : header.fields)
                {
                    const char *base_ptr = buffer.get() + field.offset * header.points;
                    if (field.name == "x")
                    {
                        for (int i = 0; i < header.points; i++)
                        {
                            pointcloud.points_[i](0) = UnpackBinaryPCDElement(
                                base_ptr + i * field.size * field.count, field.type,
                                field.size);
                        }
                    }
                    else if (field.name == "y")
                    {
                        for (int i = 0; i < header.points; i++)
                        {
                            pointcloud.points_[i](1) = UnpackBinaryPCDElement(
                                base_ptr + i * field.size * field.count, field.type,
                                field.size);
                        }
                    }
                    else if (field.name == "z")
                    {
                        for (int i = 0; i < header.points; i++)
                        {
                            pointcloud.points_[i](2) = UnpackBinaryPCDElement(
                                base_ptr + i * field.size * field.count, field.type,
                                field.size);
                        }
                    }
                    else if (field.name == "intensity")
                    {
                        for (int i = 0; i < header.points; i++)
                        {
                            pointcloud.intensitys_[i] = UnpackBinaryPCDElement(
                                base_ptr + i * field.size * field.count, field.type,
                                field.size);
                        }
                    }
                    else if (field.name == "normal_x")
                    {
                        for (int i = 0; i < header.points; i++)
                        {
                            pointcloud.normals_[i](0) = UnpackBinaryPCDElement(
                                base_ptr + i * field.size * field.count, field.type,
                                field.size);
                        }
                    }
                    else if (field.name == "normal_y")
                    {
                        for (int i = 0; i < header.points; i++)
                        {
                            pointcloud.normals_[i](1) = UnpackBinaryPCDElement(
                                base_ptr + i * field.size * field.count, field.type,
                                field.size);
                        }
                    }
                    else if (field.name == "normal_z")
                    {
                        for (int i = 0; i < header.points; i++)
                        {
                            pointcloud.normals_[i](2) = UnpackBinaryPCDElement(
                                base_ptr + i * field.size * field.count, field.type,
                                field.size);
                        }
                    }
                    else if (field.name == "rgb" || field.name == "rgba")
                    {
                        for (int i = 0; i < header.points; i++)
                        {
                            pointcloud.colors_[i] = UnpackBinaryPCDColor(
                                base_ptr + i * field.size * field.count, field.type,
                                field.size);
                        }
                    }
                }
            }
            return true;
        }

        bool GenerateHeader(const geometry::PointCloud &pointcloud,
                            const bool write_ascii,
                            const bool compressed,
                            PCDHeader &header)
        {
            if (!pointcloud.HasPoints())
            {
                return false;
            }
            header.version = "0.7";
            header.width = (int)pointcloud.points_.size();
            header.height = 1;
            header.points = header.width;
            header.fields.clear();
            PCLPointField field;
            field.type = 'F';
            field.size = 4;
            field.count = 1;
            field.name = "x";
            header.fields.push_back(field);
            field.name = "y";
            header.fields.push_back(field);
            field.name = "z";
            header.fields.push_back(field);
            header.elementnum = 3;
            header.pointsize = 12;
            if (pointcloud.HasNormals())
            {
                field.name = "normal_x";
                header.fields.push_back(field);
                field.name = "normal_y";
                header.fields.push_back(field);
                field.name = "normal_z";
                header.fields.push_back(field);
                header.elementnum += 3;
                header.pointsize += 12;
            }
            if (pointcloud.HasColors())
            {
                field.name = "rgb";
                header.fields.push_back(field);
                header.elementnum++;
                header.pointsize += 4;
            }
            if (pointcloud.HasIntensitys())
            {
                field.name = "intensity";
                header.fields.push_back(field);
                header.elementnum++;
                header.pointsize += 4;
            }
            if (write_ascii)
            {
                header.datatype = PCD_DATA_ASCII;
            }
            else
            {
                if (compressed)
                {
                    header.datatype = PCD_DATA_BINARY_COMPRESSED;
                }
                else
                {
                    header.datatype = PCD_DATA_BINARY;
                }
            }
            return true;
        }

        bool WritePCDHeader(FILE *file, const PCDHeader &header)
        {
            fprintf(file, "# .PCD v%s - Point Cloud Data file format\n",
                    header.version.c_str());
            fprintf(file, "VERSION %s\n", header.version.c_str());
            fprintf(file, "FIELDS");
            for (const auto &field : header.fields)
            {
                fprintf(file, " %s", field.name.c_str());
            }
            fprintf(file, "\n");
            fprintf(file, "SIZE");
            for (const auto &field : header.fields)
            {
                fprintf(file, " %d", field.size);
            }
            fprintf(file, "\n");
            fprintf(file, "TYPE");
            for (const auto &field : header.fields)
            {
                fprintf(file, " %c", field.type);
            }
            fprintf(file, "\n");
            fprintf(file, "COUNT");
            for (const auto &field : header.fields)
            {
                fprintf(file, " %d", field.count);
            }
            fprintf(file, "\n");
            fprintf(file, "WIDTH %d\n", header.width);
            fprintf(file, "HEIGHT %d\n", header.height);
            fprintf(file, "VIEWPOINT 0 0 0 1 0 0 0\n");
            fprintf(file, "POINTS %d\n", header.points);

            switch (header.datatype)
            {
            case PCD_DATA_BINARY:
                fprintf(file, "DATA binary\n");
                break;
            case PCD_DATA_BINARY_COMPRESSED:
                fprintf(file, "DATA binary_compressed\n");
                break;
            case PCD_DATA_ASCII:
            default:
                fprintf(file, "DATA ascii\n");
                break;
            }
            return true;
        }

        float ConvertRGBToFloat(const Eigen::Vector3d &color)
        {
            auto rgb = ColorToUint8(color);
            std::uint8_t rgba[4] = {rgb(2), rgb(1), rgb(0), 0};
            float value;
            memcpy(&value, rgba, 4);
            return value;
        }

        bool WritePCDData(FILE *file,
                          const PCDHeader &header,
                          const geometry::PointCloud &pointcloud,
                          const WritePointCloudOption &params)
        {
            bool has_intensity = pointcloud.HasIntensitys();
            bool has_normal = pointcloud.HasNormals();
            bool has_color = pointcloud.HasColors();
            if (header.datatype == PCD_DATA_ASCII)
            {
                for (size_t i = 0; i < pointcloud.points_.size(); i++)
                {
                    const auto &point = pointcloud.points_[i];
                    fprintf(file, "%.10g %.10g %.10g\n", point(0), point(1), point(2));
                    if (has_intensity)
                    {
                        const auto &intensity = pointcloud.intensitys_[i];
                        fprintf(file, " %.1f\n", intensity);
                    }
                    if (has_normal)
                    {
                        const auto &normal = pointcloud.normals_[i];
                        fprintf(file, " %.10g %.10g %.10g\n", normal(0), normal(1),
                                normal(2));
                    }
                    if (has_color)
                    {
                        const auto &color = pointcloud.colors_[i];
                        fprintf(file, " %.10g\n", ConvertRGBToFloat(color));
                    }
                    fprintf(file, "\n");
                }
            }
            else if (header.datatype == PCD_DATA_BINARY)
            {
                std::unique_ptr<float[]> data(new float[header.elementnum]);
                for (size_t i = 0; i < pointcloud.points_.size(); i++)
                {
                    const auto &point = pointcloud.points_[i];
                    data[0] = (float)point(0);
                    data[1] = (float)point(1);
                    data[2] = (float)point(2);
                    int idx = 3;
                    if (has_intensity)
                    {
                        const auto &intensity = pointcloud.intensitys_[i];
                        data[idx] = intensity;
                        idx += 1;
                    }
                    if (has_normal)
                    {
                        const auto &normal = pointcloud.normals_[i];
                        data[idx + 0] = (float)normal(0);
                        data[idx + 1] = (float)normal(1);
                        data[idx + 2] = (float)normal(2);
                        idx += 3;
                    }
                    if (has_color)
                    {
                        const auto &color = pointcloud.colors_[i];
                        data[idx] = ConvertRGBToFloat(color);
                    }
                    fwrite(data.get(), sizeof(float), header.elementnum, file);
                }
            }
            else if (header.datatype == PCD_DATA_BINARY_COMPRESSED)
            {
                double report_total = double(pointcloud.points_.size() * 2);
                // 0%-50% packing into buffer
                // 50%-75% compressing buffer
                // 75%-100% writing compressed buffer
                int strip_size = header.points;
                std::uint32_t buffer_size =
                    (std::uint32_t)(header.elementnum * header.points);
                std::unique_ptr<float[]> buffer(new float[buffer_size]);
                std::unique_ptr<float[]> buffer_compressed(new float[buffer_size * 2]);
                for (size_t i = 0; i < pointcloud.points_.size(); i++)
                {
                    const auto &point = pointcloud.points_[i];
                    buffer[0 * strip_size + i] = (float)point(0);
                    buffer[1 * strip_size + i] = (float)point(1);
                    buffer[2 * strip_size + i] = (float)point(2);
                    int idx = 3;
                    if (has_intensity)
                    {
                        const auto &intensity = pointcloud.intensitys_[i];
                        buffer[idx * strip_size + i] = intensity;
                        idx += 1;
                    }
                    if (has_normal)
                    {
                        const auto &normal = pointcloud.normals_[i];
                        buffer[(idx + 0) * strip_size + i] = (float)normal(0);
                        buffer[(idx + 1) * strip_size + i] = (float)normal(1);
                        buffer[(idx + 2) * strip_size + i] = (float)normal(2);
                        idx += 3;
                    }
                    if (has_color)
                    {
                        const auto &color = pointcloud.colors_[i];
                        buffer[idx * strip_size + i] = ConvertRGBToFloat(color);
                    }
                }
                std::uint32_t buffer_size_in_bytes = buffer_size * sizeof(float);
                std::uint32_t size_compressed =
                    lzfCompress(buffer.get(), buffer_size_in_bytes,
                                buffer_compressed.get(), buffer_size_in_bytes * 2);
                if (size_compressed == 0)
                {
                    fprintf(stderr, "[WritePCDData] Failed to compress data.\n");
                    return false;
                }
                fprintf(stderr, "[WritePCDData] %d bytes data compressed into %d bytes.\n",
                        buffer_size_in_bytes, size_compressed);
                fwrite(&size_compressed, sizeof(size_compressed), 1, file);
                fwrite(&buffer_size_in_bytes, sizeof(buffer_size_in_bytes), 1, file);
                fwrite(buffer_compressed.get(), 1, size_compressed, file);
            }
            return true;
        }

    } // unnamed namespace

    namespace io
    {
        // FileGeometry ReadFileGeometryTypePCD(const std::string &path)
        // {
        //     return CONTAINS_POINTS;
        // }

        bool ReadPointCloudFromPCD(const std::string &filename,
                                   geometry::PointCloud &pointcloud)
        {
            PCDHeader header;
            FILE *file = fopen(filename.c_str(), "rb");
            if (file == NULL)
            {
                fprintf(stderr, "Read PCD failed: unable to open file: %s\n", filename.c_str());
                return false;
            }
            if (!ReadPCDHeader(file, header))
            {
                fprintf(stderr, "Read PCD failed: unable to parse header.\n");
                fclose(file);
                return false;
            }
            fprintf(stderr, "PCD header indicates %d fields, %d bytes per point, and %d points in total.\n",
                    (int)header.fields.size(), header.pointsize, header.points);
            for (const auto &field : header.fields)
            {
                fprintf(stderr, "%s, %c, %d, %d, %d\n", field.name.c_str(),
                        field.type, field.size, field.count, field.offset);
            }
            fprintf(stderr, "Compression method is %d.\n", (int)header.datatype);
            fprintf(stderr, "Points: %s;  intensitys: %s;  normals: %s;  colors: %s\n",
                    header.has_intensitys ? "yes" : "no",
                    header.has_points ? "yes" : "no",
                    header.has_normals ? "yes" : "no",
                    header.has_colors ? "yes" : "no");
            if (!ReadPCDData(file, header, pointcloud))
            {
                fprintf(stderr, "Read PCD failed: unable to read data.\n");
                fclose(file);
                return false;
            }
            fclose(file);
            return true;
        }

        bool WritePointCloudToPCD(const std::string &filename,
                                  const geometry::PointCloud &pointcloud,
                                  const WritePointCloudOption &params)
        {
            PCDHeader header;
            if (!GenerateHeader(pointcloud, bool(params.write_ascii),
                                bool(params.compressed), header))
            {
                fprintf(stderr, "Write PCD failed: unable to generate header.\n");
                return false;
            }
            FILE *file = fopen(filename.c_str(), "wb");
            if (file == NULL)
            {
                fprintf(stderr, "Write PCD failed: unable to open file.\n");
                return false;
            }
            if (!WritePCDHeader(file, header))
            {
                fprintf(stderr, "Write PCD failed: unable to write header.\n");
                fclose(file);
                return false;
            }
            if (!WritePCDData(file, header, pointcloud, params))
            {
                fprintf(stderr, "Write PCD failed: unable to write data.\n");
                fclose(file);
                return false;
            }
            fclose(file);
            return true;
        }

    } // namespace io
} // namespace open3d
