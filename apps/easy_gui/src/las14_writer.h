#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace easy_gui
{
    //! One named extra-bytes attribute (LAS 1.4 "Extra Bytes" VLR, record id 4).
    struct ExtraBytesAttribute
    {
        enum class Type : uint8_t
        {
            U8 = 1, // LAS 1.4 data_type codes
            F32 = 9
        };
        Type type;
        std::string name; // max 32 characters
        std::string description; // max 32 characters
    };

    struct Las14Point
    {
        Eigen::Vector3d position; // in the file's coordinate frame, before header offset/scale
        double gps_time = 0.0;
        uint16_t intensity = 0;
        uint16_t point_source_id = 0;
        uint8_t user_data = 0;
        const uint8_t* extra_bytes = nullptr; // packed attribute values in declaration order, or null for all zeros
    };

    //! Streaming LAZ writer for LAS 1.4, point format 6, with extra-bytes attributes. Points are written one by one;
    //! header counts and bounds are filled in at close() (laszip inventory), so nothing is buffered here.
    //!
    //! Global encoding: bit 0 (GPS time type) = 0, because gps_time holds Unix epoch seconds, not adjusted standard
    //! GPS time; bit 4 (WKT) = 1, which LAS 1.4 requires for point formats 6-10 (no CRS record is written).
    class Las14Writer
    {
    public:
        Las14Writer() = default;
        ~Las14Writer();
        Las14Writer(const Las14Writer&) = delete;
        Las14Writer& operator=(const Las14Writer&) = delete;

        //! `scale` is the coordinate resolution in metres; `offset` the header offset.
        bool open(
            const std::string& file_name,
            const std::vector<ExtraBytesAttribute>& attributes,
            const Eigen::Vector3d& offset,
            double scale,
            const std::string& system_identifier,
            const std::string& generating_software,
            std::string& error);
        bool write(const Las14Point& point, std::string& error);
        bool close(std::string& error);

        uint64_t pointsWritten() const
        {
            return points_written_;
        }
        size_t extraBytesSize() const
        {
            return extra_bytes_size_;
        }

    private:
        void* writer_ = nullptr; // laszip_POINTER
        void* point_ = nullptr; // laszip_point*
        size_t extra_bytes_size_ = 0;
        uint64_t points_written_ = 0;
    };
} // namespace easy_gui
