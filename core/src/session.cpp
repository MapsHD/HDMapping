#include <session.h>
#include <nlohmann/json.hpp>

bool Sessionsave(const std::string &file_name){
    nlohmann::json jj;
    nlohmann::json j;

    // ImGui::Text("Offset x: %.10f y: %.10f z: %.10f", session.point_clouds_container.offset.x(), session.point_clouds_container.offset.y(), session.point_clouds_container.offset.z());


    //j["shift_x"] = common_data.shift_x;
    //j["shift_y"] = common_data.shift_y;

    //j["background_color_x"] = common_data.background_color.x;
    //j["background_color_y"] = common_data.background_color.y;
    //j["background_color_z"] = common_data.background_color.z;
    //j["background_color_w"] = common_data.background_color.w;

    return true;
}