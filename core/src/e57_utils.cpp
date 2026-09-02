#include <pch/pch.h>

#include <Core/e57_utils.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

#include <spdlog/spdlog.h>

#include <E57Exception.h>
#include <E57SimpleReader.h>
#include <E57SimpleWriter.h>

namespace mandeye::e57io
{
    namespace
    {
        constexpr size_t kChunk = 1u << 20; // points per read() call

        Eigen::Affine3d pose_from_header(const ::e57::Data3D& h)
        {
            const auto& r = h.pose.rotation;
            const auto& t = h.pose.translation;
            Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
            if (q.norm() < 1e-12)
                q = Eigen::Quaterniond::Identity();
            q.normalize();

            Eigen::Affine3d m = Eigen::Affine3d::Identity();
            m.linear() = q.toRotationMatrix();
            m.translation() = Eigen::Vector3d(t.x, t.y, t.z);
            return m;
        }

        // Maps a raw value in [lo, hi] to [0, 1]. Falls back to `assume_max` when the
        // header gives no usable limits (lo == hi), which is common for color (0..255).
        double normalize(double v, double lo, double hi, double assume_max)
        {
            if (hi > lo)
                return std::clamp((v - lo) / (hi - lo), 0.0, 1.0);
            if (assume_max > 0.0)
                return std::clamp(v / assume_max, 0.0, 1.0);
            return std::clamp(v, 0.0, 1.0);
        }

        ::e57::RigidBodyTransform rbt_from_affine(const Eigen::Affine3d& m)
        {
            Eigen::Quaterniond q(m.rotation());
            q.normalize();

            ::e57::RigidBodyTransform t;
            t.rotation.w = q.w();
            t.rotation.x = q.x();
            t.rotation.y = q.y();
            t.rotation.z = q.z();
            t.translation.x = m.translation().x();
            t.translation.y = m.translation().y();
            t.translation.z = m.translation().z();
            return t;
        }

        // Chunk-sized storage for every point field a Data3D can carry, with the
        // matching Data3DPointsDouble pointers wired up. One instance is bound to
        // both a reader and a writer so points stream through verbatim.
        struct CopyBuffers
        {
            std::vector<double> cx, cy, cz, sr, sa, se, intensity, tstamp;
            std::vector<uint16_t> cr, cg, cb;
            std::vector<int32_t> rowIdx, colIdx;
            std::vector<int8_t> cartInv, sphInv, iInv, cInv, tInv, retIdx, retCnt;
            std::vector<float> nx, ny, nz;

            void bind(const ::e57::Data3D& h, size_t chunk, ::e57::Data3DPointsDouble& b)
            {
                const auto& f = h.pointFields;
                auto dbl = [chunk](bool on, std::vector<double>& v, double*& p)
                {
                    if (on)
                    {
                        v.assign(chunk, 0.0);
                        p = v.data();
                    }
                };
                auto u16 = [chunk](bool on, std::vector<uint16_t>& v, uint16_t*& p)
                {
                    if (on)
                    {
                        v.assign(chunk, 0);
                        p = v.data();
                    }
                };
                auto i32 = [chunk](bool on, std::vector<int32_t>& v, int32_t*& p)
                {
                    if (on)
                    {
                        v.assign(chunk, 0);
                        p = v.data();
                    }
                };
                auto i8 = [chunk](bool on, std::vector<int8_t>& v, int8_t*& p)
                {
                    if (on)
                    {
                        v.assign(chunk, 0);
                        p = v.data();
                    }
                };
                auto flt = [chunk](bool on, std::vector<float>& v, float*& p)
                {
                    if (on)
                    {
                        v.assign(chunk, 0.0f);
                        p = v.data();
                    }
                };

                dbl(f.cartesianXField, cx, b.cartesianX);
                dbl(f.cartesianYField, cy, b.cartesianY);
                dbl(f.cartesianZField, cz, b.cartesianZ);
                i8(f.cartesianInvalidStateField, cartInv, b.cartesianInvalidState);
                dbl(f.sphericalRangeField, sr, b.sphericalRange);
                dbl(f.sphericalAzimuthField, sa, b.sphericalAzimuth);
                dbl(f.sphericalElevationField, se, b.sphericalElevation);
                i8(f.sphericalInvalidStateField, sphInv, b.sphericalInvalidState);
                dbl(f.intensityField, intensity, b.intensity);
                i8(f.isIntensityInvalidField, iInv, b.isIntensityInvalid);
                u16(f.colorRedField, cr, b.colorRed);
                u16(f.colorGreenField, cg, b.colorGreen);
                u16(f.colorBlueField, cb, b.colorBlue);
                i8(f.isColorInvalidField, cInv, b.isColorInvalid);
                i32(f.rowIndexField, rowIdx, b.rowIndex);
                i32(f.columnIndexField, colIdx, b.columnIndex);
                i8(f.returnIndexField, retIdx, b.returnIndex);
                i8(f.returnCountField, retCnt, b.returnCount);
                dbl(f.timeStampField, tstamp, b.timeStamp);
                i8(f.isTimeStampInvalidField, tInv, b.isTimeStampInvalid);
                flt(f.normalXField, nx, b.normalX);
                flt(f.normalYField, ny, b.normalY);
                flt(f.normalZField, nz, b.normalZ);
            }
        };

        // `pose_deltas` maps a Data3D guid to the rigid transform (new * old^-1)
        // applied to that scan, so 2D images rigidly attached to a moved scan
        // follow it.
        void copy_image2d(
            const ::e57::Reader& reader, ::e57::Writer& writer, int64_t srcIdx, const std::map<std::string, Eigen::Affine3d>& pose_deltas)
        {
            ::e57::Image2D ih;
            if (!reader.ReadImage2D(srcIdx, ih))
                throw std::runtime_error("failed to read Image2D header " + std::to_string(srcIdx));

            const auto delta = pose_deltas.find(ih.associatedData3DGuid);
            if (delta != pose_deltas.end())
            {
                const auto& r = ih.pose.rotation;
                const auto& t = ih.pose.translation;
                Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
                if (q.norm() < 1e-12)
                    q = Eigen::Quaterniond::Identity();
                q.normalize();
                Eigen::Affine3d img = Eigen::Affine3d::Identity();
                img.linear() = q.toRotationMatrix();
                img.translation() = Eigen::Vector3d(t.x, t.y, t.z);

                ih.pose = rbt_from_affine(delta->second * img);
            }

            const int64_t dstIdx = writer.NewImage2D(ih);

            auto copyRepr = [&](::e57::Image2DProjection proj, int64_t jpegSize, int64_t pngSize, int64_t maskSize)
            {
                auto blob = [&](::e57::Image2DType type, int64_t size)
                {
                    if (size <= 0)
                        return;
                    std::vector<uint8_t> buf(static_cast<size_t>(size));
                    const int64_t got = reader.ReadImage2DData(srcIdx, proj, type, buf.data(), 0, size);
                    if (got <= 0)
                        throw std::runtime_error("failed to read Image2D blob");
                    writer.WriteImage2DData(dstIdx, type, proj, buf.data(), 0, got);
                };
                blob(::e57::ImageJPEG, jpegSize);
                blob(::e57::ImagePNG, pngSize);
                blob(::e57::ImageMaskPNG, maskSize);
            };

            const auto& v = ih.visualReferenceRepresentation;
            const auto& p = ih.pinholeRepresentation;
            const auto& s = ih.sphericalRepresentation;
            const auto& c = ih.cylindricalRepresentation;
            if (v.jpegImageSize || v.pngImageSize || v.imageMaskSize)
                copyRepr(::e57::ProjectionVisual, v.jpegImageSize, v.pngImageSize, v.imageMaskSize);
            if (p.jpegImageSize || p.pngImageSize || p.imageMaskSize)
                copyRepr(::e57::ProjectionPinhole, p.jpegImageSize, p.pngImageSize, p.imageMaskSize);
            if (s.jpegImageSize || s.pngImageSize || s.imageMaskSize)
                copyRepr(::e57::ProjectionSpherical, s.jpegImageSize, s.pngImageSize, s.imageMaskSize);
            if (c.jpegImageSize || c.pngImageSize || c.imageMaskSize)
                copyRepr(::e57::ProjectionCylindrical, c.jpegImageSize, c.pngImageSize, c.imageMaskSize);
        }
    } // namespace

    bool load_e57(const std::string& path, std::vector<E57Scan>& scans, std::string& error)
    {
        scans.clear();
        error.clear();

        try
        {
            ::e57::Reader reader(path, ::e57::ReaderOptions{});
            if (!reader.IsOpen())
            {
                error = "could not open E57 file: '" + path + "'";
                return false;
            }

            const int64_t scan_count = reader.GetData3DCount();
            if (scan_count <= 0)
            {
                error = "E57 file has no Data3D blocks: '" + path + "'";
                return false;
            }

            for (int64_t si = 0; si < scan_count; si++)
            {
                ::e57::Data3D header;
                if (!reader.ReadData3D(si, header))
                {
                    spdlog::warn("e57: failed to read Data3D header {} in '{}', skipping", si, path);
                    continue;
                }

                int64_t row_max = 0, col_max = 0, points_size = 0, groups_size = 0, count_size = 0;
                bool column_index = false;
                reader.GetData3DSizes(si, row_max, col_max, points_size, groups_size, count_size, column_index);
                if (points_size <= 0)
                    points_size = static_cast<int64_t>(header.pointCount);
                if (points_size <= 0)
                {
                    spdlog::warn("e57: Data3D {} in '{}' has no points, skipping", si, path);
                    continue;
                }

                const auto& pf = header.pointFields;
                const bool has_cartesian = pf.cartesianXField && pf.cartesianYField && pf.cartesianZField;
                const bool has_spherical = pf.sphericalRangeField && pf.sphericalAzimuthField && pf.sphericalElevationField;
                if (!has_cartesian && !has_spherical)
                {
                    spdlog::warn("e57: Data3D {} in '{}' has neither cartesian nor spherical coords, skipping", si, path);
                    continue;
                }
                const bool has_intensity = pf.intensityField;
                const bool has_color = pf.colorRedField && pf.colorGreenField && pf.colorBlueField;
                const bool has_time = pf.timeStampField;

                const size_t chunk = std::min<size_t>(kChunk, static_cast<size_t>(points_size));

                std::vector<double> cx, cy, cz, sr, sa, se, intensity, tstamp;
                std::vector<uint16_t> cr, cg, cb;
                std::vector<int8_t> cart_invalid, sph_invalid, intensity_invalid, color_invalid;

                ::e57::Data3DPointsDouble buffers; // default ctor: does not own memory
                if (has_cartesian)
                {
                    cx.resize(chunk);
                    cy.resize(chunk);
                    cz.resize(chunk);
                    buffers.cartesianX = cx.data();
                    buffers.cartesianY = cy.data();
                    buffers.cartesianZ = cz.data();
                    if (pf.cartesianInvalidStateField)
                    {
                        cart_invalid.resize(chunk);
                        buffers.cartesianInvalidState = cart_invalid.data();
                    }
                }
                if (has_spherical)
                {
                    sr.resize(chunk);
                    sa.resize(chunk);
                    se.resize(chunk);
                    buffers.sphericalRange = sr.data();
                    buffers.sphericalAzimuth = sa.data();
                    buffers.sphericalElevation = se.data();
                    if (pf.sphericalInvalidStateField)
                    {
                        sph_invalid.resize(chunk);
                        buffers.sphericalInvalidState = sph_invalid.data();
                    }
                }
                if (has_intensity)
                {
                    intensity.resize(chunk);
                    buffers.intensity = intensity.data();
                    if (pf.isIntensityInvalidField)
                    {
                        intensity_invalid.resize(chunk);
                        buffers.isIntensityInvalid = intensity_invalid.data();
                    }
                }
                if (has_color)
                {
                    cr.resize(chunk);
                    cg.resize(chunk);
                    cb.resize(chunk);
                    buffers.colorRed = cr.data();
                    buffers.colorGreen = cg.data();
                    buffers.colorBlue = cb.data();
                    if (pf.isColorInvalidField)
                    {
                        color_invalid.resize(chunk);
                        buffers.isColorInvalid = color_invalid.data();
                    }
                }
                if (has_time)
                {
                    tstamp.resize(chunk);
                    buffers.timeStamp = tstamp.data();
                }

                E57Scan scan;
                scan.name = !header.name.empty() ? header.name : ("scan_" + std::to_string(si));
                scan.pose = pose_from_header(header);
                scan.points.reserve(static_cast<size_t>(points_size));
                if (has_intensity)
                    scan.intensities.reserve(static_cast<size_t>(points_size));
                if (has_color)
                    scan.colors.reserve(static_cast<size_t>(points_size));
                if (has_time)
                    scan.timestamps.reserve(static_cast<size_t>(points_size));

                const double i_lo = header.intensityLimits.intensityMinimum;
                const double i_hi = header.intensityLimits.intensityMaximum;
                const double cr_lo = header.colorLimits.colorRedMinimum;
                const double cr_hi = header.colorLimits.colorRedMaximum;
                const double cg_lo = header.colorLimits.colorGreenMinimum;
                const double cg_hi = header.colorLimits.colorGreenMaximum;
                const double cb_lo = header.colorLimits.colorBlueMinimum;
                const double cb_hi = header.colorLimits.colorBlueMaximum;

                ::e57::CompressedVectorReader vr = reader.SetUpData3DPointsData(si, chunk, buffers);
                unsigned got = 0;
                while ((got = vr.read()) > 0)
                {
                    for (unsigned k = 0; k < got; k++)
                    {
                        if (!cart_invalid.empty() && cart_invalid[k] != 0)
                            continue;
                        if (!sph_invalid.empty() && sph_invalid[k] != 0)
                            continue;

                        Eigen::Vector3d p;
                        if (has_cartesian)
                        {
                            p = Eigen::Vector3d(cx[k], cy[k], cz[k]);
                        }
                        else
                        {
                            const double rng = sr[k];
                            const double az = sa[k];
                            const double el = se[k];
                            p = Eigen::Vector3d(rng * std::cos(el) * std::cos(az), rng * std::cos(el) * std::sin(az), rng * std::sin(el));
                        }
                        if (!p.allFinite())
                            continue;

                        scan.points.push_back(p);

                        if (has_intensity)
                        {
                            const double n = normalize(intensity[k], i_lo, i_hi, 0.0);
                            scan.intensities.push_back(static_cast<unsigned short>(std::lround(n * 65535.0)));
                        }
                        if (has_color)
                        {
                            scan.colors.emplace_back(
                                normalize(cr[k], cr_lo, cr_hi, 255.0),
                                normalize(cg[k], cg_lo, cg_hi, 255.0),
                                normalize(cb[k], cb_lo, cb_hi, 255.0));
                        }
                        if (has_time)
                            scan.timestamps.push_back(tstamp[k]);
                    }
                }
                vr.close();

                if (scan.points.empty())
                {
                    spdlog::warn("e57: Data3D {} ('{}') in '{}' produced no valid points, skipping", si, scan.name, path);
                    continue;
                }

                spdlog::info(
                    "e57: loaded scan '{}' ({} points{}{}{}) from '{}'",
                    scan.name,
                    scan.points.size(),
                    has_intensity ? ", intensity" : "",
                    has_color ? ", rgb" : "",
                    has_time ? ", time" : "",
                    path);
                scans.push_back(std::move(scan));
            }
        } catch (const ::e57::E57Exception& e)
        {
            error = "E57 error while reading '" + path + "': " + std::string(e.what()) + " (" + e.context() + ")";
            scans.clear();
            return false;
        } catch (const std::exception& e)
        {
            error = "error while reading '" + path + "': " + e.what();
            scans.clear();
            return false;
        }

        if (scans.empty())
        {
            if (error.empty())
                error = "no readable scans in '" + path + "'";
            return false;
        }
        return true;
    }

    bool rewrite_e57_poses(
        const std::string& src_path, const std::string& dst_path, const std::map<int, Eigen::Affine3d>& new_poses, std::string& error)
    {
        error.clear();
        if (src_path == dst_path)
        {
            error = "rewrite_e57_poses: source and destination path must differ";
            return false;
        }

        try
        {
            ::e57::Reader reader(src_path, ::e57::ReaderOptions{});
            if (!reader.IsOpen())
            {
                error = "could not open '" + src_path + "'";
                return false;
            }

            ::e57::E57Root root;
            reader.GetE57Root(root);

            ::e57::WriterOptions wopts;
            wopts.coordinateMetadata = root.coordinateMetadata; // fresh file guid on purpose
            ::e57::Writer writer(dst_path, wopts);
            if (!writer.IsOpen())
            {
                error = "could not create '" + dst_path + "'";
                return false;
            }

            std::map<std::string, Eigen::Affine3d> pose_deltas; // Data3D guid -> new * old^-1

            const int64_t scan_count = reader.GetData3DCount();
            for (int64_t i = 0; i < scan_count; i++)
            {
                ::e57::Data3D header;
                if (!reader.ReadData3D(i, header))
                    throw std::runtime_error("failed to read Data3D header " + std::to_string(i));

                int64_t row_max = 0, col_max = 0, points_size = 0, groups_size = 0, count_size = 0;
                bool column_index = false;
                reader.GetData3DSizes(i, row_max, col_max, points_size, groups_size, count_size, column_index);
                if (points_size <= 0)
                    points_size = static_cast<int64_t>(header.pointCount);
                const size_t total = points_size > 0 ? static_cast<size_t>(points_size) : 0;

                const auto it = new_poses.find(static_cast<int>(i));
                if (it != new_poses.end())
                {
                    const Eigen::Affine3d old_pose = pose_from_header(header);
                    header.pose = rbt_from_affine(it->second);
                    if (!header.guid.empty())
                        pose_deltas[header.guid] = it->second * old_pose.inverse();
                    spdlog::info("e57: updating pose of Data3D {} ('{}') in '{}'", i, header.name, src_path);
                }

                header.pointCount = total;
                const int64_t di = writer.NewData3D(header);

                if (total > 0)
                {
                    const size_t chunk = std::min<size_t>(kChunk, total);

                    ::e57::Data3DPointsDouble buffers;
                    CopyBuffers storage;
                    storage.bind(header, chunk, buffers);

                    ::e57::CompressedVectorReader cvr = reader.SetUpData3DPointsData(i, chunk, buffers);
                    ::e57::CompressedVectorWriter cvw = writer.SetUpData3DPointsData(di, chunk, buffers);
                    unsigned got = 0;
                    while ((got = cvr.read()) > 0)
                        cvw.write(got);
                    cvw.close();
                    cvr.close();
                }

                if (groups_size > 0)
                {
                    std::vector<int64_t> id_elem(static_cast<size_t>(groups_size));
                    std::vector<int64_t> start_idx(static_cast<size_t>(groups_size));
                    std::vector<int64_t> point_count(static_cast<size_t>(groups_size));
                    if (reader.ReadData3DGroupsData(
                            i, static_cast<size_t>(groups_size), id_elem.data(), start_idx.data(), point_count.data()))
                        writer.WriteData3DGroupsData(
                            di, static_cast<size_t>(groups_size), id_elem.data(), start_idx.data(), point_count.data());
                }
            }

            const int64_t image_count = reader.GetImage2DCount();
            for (int64_t k = 0; k < image_count; k++)
                copy_image2d(reader, writer, k, pose_deltas);

            if (!writer.Close())
            {
                error = "failed to finalize '" + dst_path + "'";
                return false;
            }
        } catch (const ::e57::E57Exception& e)
        {
            error = "E57 error while rewriting '" + src_path + "': " + std::string(e.what()) + " (" + e.context() + ")";
            return false;
        } catch (const std::exception& e)
        {
            error = "error while rewriting '" + src_path + "': " + e.what();
            return false;
        }

        return true;
    }

    bool save_e57(const std::string& dst_path, const std::vector<E57WriteScan>& scans, std::string& error)
    {
        error.clear();

        std::size_t total_written = 0;
        try
        {
            ::e57::Writer writer(dst_path, ::e57::WriterOptions{});
            if (!writer.IsOpen())
            {
                error = "could not create '" + dst_path + "'";
                return false;
            }

            for (std::size_t si = 0; si < scans.size(); si++)
            {
                const auto& s = scans[si];
                if (s.points == nullptr || s.points->empty())
                {
                    spdlog::warn("save_e57: scan {} ('{}') has no points, skipping", si, s.name);
                    continue;
                }

                const std::size_t n = s.points->size();
                const bool has_i = s.intensities != nullptr && s.intensities->size() == n;
                const bool has_c = s.colors != nullptr && s.colors->size() == n;
                const bool has_t = s.timestamps != nullptr && s.timestamps->size() == n;

                ::e57::Data3D header;
                header.name = s.name.empty() ? ("scan_" + std::to_string(si)) : s.name;
                header.description = s.description;
                header.pointCount = n;
                header.pose = rbt_from_affine(s.pose);

                auto& pf = header.pointFields;
                pf.cartesianXField = true;
                pf.cartesianYField = true;
                pf.cartesianZField = true;
                pf.pointRangeNodeType = ::e57::NumericalNodeType::Double;
                if (has_i)
                {
                    pf.intensityField = true;
                    pf.intensityNodeType = ::e57::NumericalNodeType::Float;
                    header.intensityLimits = { 0.0, 65535.0 };
                }
                if (has_c)
                {
                    pf.colorRedField = true;
                    pf.colorGreenField = true;
                    pf.colorBlueField = true;
                    header.colorLimits = { 0.0, 255.0, 0.0, 255.0, 0.0, 255.0 };
                }
                if (has_t)
                {
                    pf.timeStampField = true;
                    pf.timeNodeType = ::e57::NumericalNodeType::Double;
                }

                Eigen::Vector3d lo = (*s.points)[0];
                Eigen::Vector3d hi = (*s.points)[0];
                for (const auto& p : *s.points)
                {
                    lo = lo.cwiseMin(p);
                    hi = hi.cwiseMax(p);
                }
                header.cartesianBounds.xMinimum = lo.x();
                header.cartesianBounds.yMinimum = lo.y();
                header.cartesianBounds.zMinimum = lo.z();
                header.cartesianBounds.xMaximum = hi.x();
                header.cartesianBounds.yMaximum = hi.y();
                header.cartesianBounds.zMaximum = hi.z();

                const int64_t di = writer.NewData3D(header);

                const std::size_t chunk = std::min<std::size_t>(kChunk, n);
                std::vector<double> cx(chunk), cy(chunk), cz(chunk), inten, tim;
                std::vector<uint16_t> cr, cg, cb;

                ::e57::Data3DPointsDouble buffers;
                buffers.cartesianX = cx.data();
                buffers.cartesianY = cy.data();
                buffers.cartesianZ = cz.data();
                if (has_i)
                {
                    inten.resize(chunk);
                    buffers.intensity = inten.data();
                }
                if (has_c)
                {
                    cr.resize(chunk);
                    cg.resize(chunk);
                    cb.resize(chunk);
                    buffers.colorRed = cr.data();
                    buffers.colorGreen = cg.data();
                    buffers.colorBlue = cb.data();
                }
                if (has_t)
                {
                    tim.resize(chunk);
                    buffers.timeStamp = tim.data();
                }

                ::e57::CompressedVectorWriter vw = writer.SetUpData3DPointsData(di, chunk, buffers);
                std::size_t done = 0;
                while (done < n)
                {
                    const std::size_t m = std::min(chunk, n - done);
                    for (std::size_t k = 0; k < m; k++)
                    {
                        const Eigen::Vector3d& p = (*s.points)[done + k];
                        cx[k] = p.x();
                        cy[k] = p.y();
                        cz[k] = p.z();
                        if (has_i)
                            inten[k] = static_cast<double>((*s.intensities)[done + k]);
                        if (has_c)
                        {
                            const Eigen::Vector3d& col = (*s.colors)[done + k];
                            cr[k] = static_cast<uint16_t>(std::lround(std::clamp(col.x(), 0.0, 1.0) * 255.0));
                            cg[k] = static_cast<uint16_t>(std::lround(std::clamp(col.y(), 0.0, 1.0) * 255.0));
                            cb[k] = static_cast<uint16_t>(std::lround(std::clamp(col.z(), 0.0, 1.0) * 255.0));
                        }
                        if (has_t)
                            tim[k] = (*s.timestamps)[done + k];
                    }
                    vw.write(m);
                    done += m;
                }
                vw.close();

                total_written++;
                spdlog::info(
                    "save_e57: wrote scan '{}' ({} points{}{}{}) to '{}'",
                    header.name,
                    n,
                    has_i ? ", intensity" : "",
                    has_c ? ", rgb" : "",
                    has_t ? ", time" : "",
                    dst_path);
            }

            if (!writer.Close())
            {
                error = "failed to finalize '" + dst_path + "'";
                return false;
            }
        } catch (const ::e57::E57Exception& e)
        {
            error = "E57 error while writing '" + dst_path + "': " + std::string(e.what()) + " (" + e.context() + ")";
            return false;
        } catch (const std::exception& e)
        {
            error = "error while writing '" + dst_path + "': " + e.what();
            return false;
        }

        if (total_written == 0)
        {
            error = "no non-empty scans to write";
            return false;
        }
        return true;
    }
} // namespace mandeye::e57io
