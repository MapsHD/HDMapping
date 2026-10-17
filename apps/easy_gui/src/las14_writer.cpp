#include "las14_writer.h"

#include <laszip/laszip_api.h>

#include <algorithm>
#include <cstring>
#include <ctime>

namespace easy_gui
{
    namespace
    {
        //! LAS 1.4 R15 "Extra Bytes" descriptor. Written by hand rather than with laszip_add_attribute(), which always
        //! sets the "scale" option bit: readers then scale the values, so laspy would return the uint8 point_origin field as
        //! float64.
#pragma pack(push, 1)
        struct ExtraBytesDescriptor
        {
            uint8_t reserved[2];
            uint8_t data_type;
            uint8_t options; // 0: no no_data/min/max/scale/offset
            char name[32];
            uint8_t unused[4];
            uint8_t no_data[24];
            uint8_t min[24];
            uint8_t max[24];
            double scale[3];
            double offset[3];
            char description[32];
        };
#pragma pack(pop)
        static_assert(sizeof(ExtraBytesDescriptor) == 192);

        size_t attributeSize(ExtraBytesAttribute::Type type)
        {
            return type == ExtraBytesAttribute::Type::U8 ? 1 : 4;
        }

        std::string laszipError(void* writer, const std::string& what)
        {
            laszip_CHAR* message = nullptr;
            if (writer && !laszip_get_error(writer, &message) && message && *message)
                return what + ": " + message;
            return what;
        }

        void copyField(char* destination, size_t size, const std::string& text)
        {
            std::memset(destination, 0, size);
            std::memcpy(destination, text.data(), std::min(size, text.size()));
        }

        constexpr uint16_t kPointFormat6RecordLength = 30;
    } // namespace

    Las14Writer::~Las14Writer()
    {
        std::string ignored;
        close(ignored);
    }

    bool Las14Writer::open(
        const std::string& file_name,
        const std::vector<ExtraBytesAttribute>& attributes,
        const Eigen::Vector3d& offset,
        double scale,
        const std::string& generating_software,
        std::string& error)
    {
        if (laszip_create(&writer_))
        {
            writer_ = nullptr;
            error = "could not create the LAZ writer";
            return false;
        }

        laszip_header* header = nullptr;
        if (laszip_get_header_pointer(writer_, &header))
        {
            error = laszipError(writer_, "could not access the LAZ header");
            return false;
        }

        extra_bytes_size_ = 0;
        for (const auto& a : attributes)
            extra_bytes_size_ += attributeSize(a.type);

        header->file_source_ID = 0;
        header->global_encoding = (1 << 4); // bit 0 = 0: not adjusted GPS time; bit 4: WKT (required for PF6)
        header->version_major = 1;
        header->version_minor = 4;
        copyField(header->system_identifier, sizeof(header->system_identifier), "HDMapping Easy GUI");
        copyField(header->generating_software, sizeof(header->generating_software), generating_software);
        {
            const std::time_t now = std::time(nullptr);
            std::tm tm{};
#ifdef _WIN32
            gmtime_s(&tm, &now);
#else
            gmtime_r(&now, &tm);
#endif
            header->file_creation_day = static_cast<laszip_U16>(tm.tm_yday + 1);
            header->file_creation_year = static_cast<laszip_U16>(tm.tm_year + 1900);
        }
        header->header_size = 375;
        header->offset_to_point_data = 375; // laszip_add_vlr() below adds the VLR sizes
        header->point_data_format = 6;
        header->point_data_record_length = static_cast<laszip_U16>(kPointFormat6RecordLength + extra_bytes_size_);
        header->x_scale_factor = header->y_scale_factor = header->z_scale_factor = scale;
        header->x_offset = offset.x();
        header->y_offset = offset.y();
        header->z_offset = offset.z();

        if (!attributes.empty())
        {
            std::vector<ExtraBytesDescriptor> descriptors(attributes.size());
            for (size_t i = 0; i < attributes.size(); i++)
            {
                ExtraBytesDescriptor& d = descriptors[i];
                std::memset(&d, 0, sizeof(d));
                d.data_type = static_cast<uint8_t>(attributes[i].type);
                copyField(d.name, sizeof(d.name), attributes[i].name);
                copyField(d.description, sizeof(d.description), attributes[i].description);
            }
            if (laszip_add_vlr(
                    writer_,
                    "LASF_Spec",
                    4,
                    static_cast<laszip_U16>(descriptors.size() * sizeof(ExtraBytesDescriptor)),
                    "extra bytes",
                    reinterpret_cast<const laszip_U8*>(descriptors.data())))
            {
                error = laszipError(writer_, "could not add the extra bytes record");
                return false;
            }
        }

        laszip_preserve_generating_software(writer_, 1);
        const laszip_BOOL compress = file_name.size() >= 4 && file_name.compare(file_name.size() - 4, 4, ".laz") == 0;
        if (laszip_open_writer(writer_, file_name.c_str(), compress))
        {
            error = laszipError(writer_, "could not open '" + file_name + "' for writing");
            return false;
        }

        laszip_point* point = nullptr;
        if (laszip_get_point_pointer(writer_, &point))
        {
            error = laszipError(writer_, "could not access the LAZ point");
            return false;
        }
        if (static_cast<size_t>(point->num_extra_bytes) != extra_bytes_size_)
        {
            error = "LAZ writer has " + std::to_string(point->num_extra_bytes) + " extra bytes per point, expected " +
                std::to_string(extra_bytes_size_);
            return false;
        }
        point->extended_point_type = 1;
        point->return_number = 1;
        point->number_of_returns = 1;
        point->extended_return_number = 1;
        point->extended_number_of_returns = 1;
        point_ = point;
        points_written_ = 0;
        return true;
    }

    bool Las14Writer::write(const Las14Point& p, std::string& error)
    {
        auto* point = static_cast<laszip_point*>(point_);
        point->gps_time = p.gps_time;
        point->intensity = p.intensity;
        point->point_source_ID = p.point_source_id;
        point->user_data = p.user_data;
        if (extra_bytes_size_ > 0)
        {
            if (p.extra_bytes)
                std::memcpy(point->extra_bytes, p.extra_bytes, extra_bytes_size_);
            else
                std::memset(point->extra_bytes, 0, extra_bytes_size_);
        }

        laszip_F64 coordinates[3] = { p.position.x(), p.position.y(), p.position.z() };
        if (laszip_set_coordinates(writer_, coordinates) || laszip_write_point(writer_) || laszip_update_inventory(writer_))
        {
            error = laszipError(writer_, "writing point " + std::to_string(points_written_));
            return false;
        }
        points_written_++;
        return true;
    }

    bool Las14Writer::close(std::string& error)
    {
        if (!writer_)
            return true;
        bool ok = true;
        if (point_ && laszip_close_writer(writer_))
        {
            error = laszipError(writer_, "could not finish the LAZ file");
            ok = false;
        }
        laszip_destroy(writer_);
        writer_ = nullptr;
        point_ = nullptr;
        return ok;
    }
} // namespace easy_gui
