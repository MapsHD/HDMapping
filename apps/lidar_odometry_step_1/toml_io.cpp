#include "toml_io.h"

bool TomlIO::SaveParametersToTomlFile(const std::string &filepath, LidarOdometryParams &params)
{
    toml::table tbl;
    for (const auto &[category, attributes] : CATEGORIES)
    {
        toml::table cat_tbl;
        for (const auto &attribute : attributes)
        {
            if (attribute == "in_out_params_indoor" || attribute == "in_out_params_outdoor")
                continue;
            
            // Check if it's a motion_model_correction parameter
            auto motion_it = MOTION_MODEL_CORRECTION_POINTERS.find(attribute);
            if (motion_it != MOTION_MODEL_CORRECTION_POINTERS.end())
            {
                cat_tbl.insert(attribute, params.motion_model_correction.*(motion_it->second));
                continue;
            }
            
            auto it = POINTERS.find(attribute);
            if (it == POINTERS.end())
                continue;
            std::visit([&](auto ptr)
                       { cat_tbl.insert(attribute, params.*ptr); }, it->second);
        }
        tbl.insert(category, std::move(cat_tbl));
    }
    // separate grid params
    auto insert_grid_params = [&](const std::string &name, const NDT::GridParameters &grid)
    {
        toml::table nested_tbl;
        nested_tbl.insert("bounding_box_min_X", grid.bounding_box_min_X);
        nested_tbl.insert("bounding_box_min_Y", grid.bounding_box_min_Y);
        nested_tbl.insert("bounding_box_min_Z", grid.bounding_box_min_Z);
        nested_tbl.insert("bounding_box_max_X", grid.bounding_box_max_X);
        nested_tbl.insert("bounding_box_max_Y", grid.bounding_box_max_Y);
        nested_tbl.insert("bounding_box_max_Z", grid.bounding_box_max_Z);
        nested_tbl.insert("bounding_box_extension", grid.bounding_box_extension);
        nested_tbl.insert("number_of_buckets_X", grid.number_of_buckets_X);
        nested_tbl.insert("number_of_buckets_Y", grid.number_of_buckets_Y);
        nested_tbl.insert("number_of_buckets_Z", grid.number_of_buckets_Z);
        nested_tbl.insert("number_of_buckets", static_cast<int64_t>(grid.number_of_buckets));
        nested_tbl.insert("resolution_X", grid.resolution_X);
        nested_tbl.insert("resolution_Y", grid.resolution_Y);
        nested_tbl.insert("resolution_Z", grid.resolution_Z);
        tbl.insert(name, std::move(nested_tbl));
    };
    insert_grid_params("in_out_params_indoor", params.in_out_params_indoor);
    insert_grid_params("in_out_params_outdoor", params.in_out_params_outdoor);
    std::ofstream out(filepath);
    out << tbl;
    return true;
}

template <typename T>
void TomlIO::set_if_exists(NDT::GridParameters &grid, const toml::table *tbl, const std::string &key, T NDT::GridParameters::*member)
{
    if (!tbl)
        return;
    if (auto it = tbl->find(key); it != tbl->end())
    {
        if (auto val = it->second.value<T>(); val.has_value())
        {
            grid.*member = *val;
        }
    }
}

void TomlIO::read_grid_params(NDT::GridParameters &grid, const toml::table *grid_tbl)
{
    set_if_exists(grid, grid_tbl, "bounding_box_min_X", &NDT::GridParameters::bounding_box_min_X);
    set_if_exists(grid, grid_tbl, "bounding_box_min_Y", &NDT::GridParameters::bounding_box_min_Y);
    set_if_exists(grid, grid_tbl, "bounding_box_min_Z", &NDT::GridParameters::bounding_box_min_Z);
    set_if_exists(grid, grid_tbl, "bounding_box_max_X", &NDT::GridParameters::bounding_box_max_X);
    set_if_exists(grid, grid_tbl, "bounding_box_max_Y", &NDT::GridParameters::bounding_box_max_Y);
    set_if_exists(grid, grid_tbl, "bounding_box_max_Z", &NDT::GridParameters::bounding_box_max_Z);
    set_if_exists(grid, grid_tbl, "bounding_box_extension", &NDT::GridParameters::bounding_box_extension);
    set_if_exists(grid, grid_tbl, "number_of_buckets_X", &NDT::GridParameters::number_of_buckets_X);
    set_if_exists(grid, grid_tbl, "number_of_buckets_Y", &NDT::GridParameters::number_of_buckets_Y);
    set_if_exists(grid, grid_tbl, "number_of_buckets_Z", &NDT::GridParameters::number_of_buckets_Z);
    if (grid_tbl)
    {
        if (auto it = grid_tbl->find("number_of_buckets"); it != grid_tbl->end())
        {
            if (auto val = it->second.value<int>(); val.has_value())
            {
                grid.number_of_buckets = static_cast<size_t>(*val);
            }
        }
    }
    set_if_exists(grid, grid_tbl, "resolution_X", &NDT::GridParameters::resolution_X);
    set_if_exists(grid, grid_tbl, "resolution_Y", &NDT::GridParameters::resolution_Y);
    set_if_exists(grid, grid_tbl, "resolution_Z", &NDT::GridParameters::resolution_Z);
}

bool TomlIO::LoadParametersFromTomlFile(const std::string &filepath, LidarOdometryParams &params)
{
    toml::table tbl;
    try
    {
        tbl = toml::parse_file(filepath);
    }
    catch (const toml::parse_error &err)
    {
        std::cerr << "Parsing failed:\n"
                  << err << "\n";
        return false;
    }
    for (const auto &[category, attributes] : CATEGORIES)
    {
        auto cat_it = tbl.find(category);
        if (cat_it == tbl.end())
            continue;
        const auto &cat_tbl = cat_it->second.as_table();
        if (!cat_tbl)
            continue;
        for (const auto &attribute : attributes)
        {
            // grid params will be processed separately
            if (attribute == "in_out_params_indoor" || attribute == "in_out_params_outdoor")
                continue;
            
            // Check if it's a motion_model_correction parameter
            auto motion_it = MOTION_MODEL_CORRECTION_POINTERS.find(attribute);
            if (motion_it != MOTION_MODEL_CORRECTION_POINTERS.end())
            {
                auto attr_it = cat_tbl->find(attribute);
                if (attr_it != cat_tbl->end())
                {
                    params.motion_model_correction.*(motion_it->second) = attr_it->second.value_or(0.0);
                }
                continue;
            }
            
            auto it = POINTERS.find(attribute);
            if (it == POINTERS.end())
                continue;
            auto attr_it = cat_tbl->find(attribute);
            if (attr_it == cat_tbl->end())
                continue;
            std::visit([&](auto ptr)
                       {
                        using MemberType = std::remove_reference_t<decltype(params.*ptr)>;
                        if constexpr (std::is_same_v<MemberType, bool>) {
                            params.*ptr = attr_it->second.value_or(false);
                        }
                        else if constexpr (std::is_same_v<MemberType, int>) {
                            params.*ptr = static_cast<int>(attr_it->second.value_or<int64_t>(0));
                        }
                        else if constexpr (std::is_same_v<MemberType, double>) {
                            params.*ptr = attr_it->second.value_or(0.0);
                        }
                        else if constexpr (std::is_same_v<MemberType, std::string>) {
                            params.*ptr = attr_it->second.value_or(std::string{});
                        } }, it->second);
        }
    }
    read_grid_params(params.in_out_params_indoor, tbl["in_out_params_indoor"].as_table());
    read_grid_params(params.in_out_params_outdoor, tbl["in_out_params_outdoor"].as_table());
    return true;
}