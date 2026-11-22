#include "bag_source_ui.h"

#include <imgui.h>

#include <cmath>

namespace
{
    std::string channel_label(const BagChannel& c)
    {
        std::string label = c.topic + "  (" + c.type;
        if (c.message_count > 0)
            label += ", " + std::to_string(c.message_count) + " msgs";
        return label + ")";
    }

    const BagChannel* find_channel(const std::vector<BagChannel>& channels, const std::string& topic)
    {
        for (const auto& c : channels)
            if (c.topic == topic)
                return &c;
        return nullptr;
    }

    //! Combo over the channels of one kind; writes the chosen topic.
    void topic_combo(const char* label, const std::vector<BagChannel>& channels, BagChannel::Kind kind, std::string& topic)
    {
        const BagChannel* current = find_channel(channels, topic);
        const std::string preview = current ? channel_label(*current) : "<choose a topic>";
        if (ImGui::BeginCombo(label, preview.c_str()))
        {
            for (const auto& c : channels)
            {
                if (c.kind != kind)
                    continue;
                const bool selected = c.topic == topic;
                if (ImGui::Selectable(channel_label(c).c_str(), selected))
                    topic = c.topic;
                if (selected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    }

    //! The topic of the only channel of this kind, or an empty string.
    std::string only_topic(const std::vector<BagChannel>& channels, BagChannel::Kind kind)
    {
        std::string topic;
        for (const auto& c : channels)
        {
            if (c.kind != kind)
                continue;
            if (!topic.empty())
                return {};
            topic = c.topic;
        }
        return topic;
    }
} // namespace

BagSourceConfigUi::BagSourceConfigUi(const std::filesystem::path& file)
{
    config_.file = file;
    channels_ = list_bag_channels(file, list_error_);
    config_.lidar_topic = only_topic(channels_, BagChannel::Kind::Lidar);
    config_.imu_topic = only_topic(channels_, BagChannel::Kind::Imu);
}

bool BagSourceConfigUi::draw()
{
    ImGui::TextUnformatted(config_.file.filename().string().c_str());
    if (!list_error_.empty())
    {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", list_error_.c_str());
        return false;
    }

    topic_combo("LiDAR topic", channels_, BagChannel::Kind::Lidar, config_.lidar_topic);
    topic_combo("IMU topic", channels_, BagChannel::Kind::Imu, config_.imu_topic);

    // the layout presets only apply to PointCloud2; a Livox CustomMsg has a fixed layout
    const BagChannel* lidar = find_channel(channels_, config_.lidar_topic);
    if (lidar && lidar->type.ends_with("PointCloud2"))
    {
        const char* layouts[] = { "auto (from the message fields)", "Hesai", "Ouster" };
        int layout = static_cast<int>(config_.lidar_layout);
        if (ImGui::Combo("PointCloud2 layout", &layout, layouts, IM_ARRAYSIZE(layouts)))
            config_.lidar_layout = static_cast<BagSourceConfig::LidarLayout>(layout);
    }

    int unit = config_.imu_acc_in_g ? 0 : 1;
    ImGui::TextUnformatted("IMU acceleration unit:");
    ImGui::SameLine();
    ImGui::RadioButton("g", &unit, 0);
    ImGui::SameLine();
    ImGui::RadioButton("m/s^2", &unit, 1);
    config_.imu_acc_in_g = unit == 0;

    ImGui::InputScalarN(
        "IMU -> lidar rotation (qx qy qz qw)", ImGuiDataType_Double, config_.imu_to_lidar.data(), 4, nullptr, nullptr, "%.6f");
    const auto& q = config_.imu_to_lidar;
    const double norm = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    const bool unit_quaternion = std::abs(norm - 1.0) < 1e-3;
    if (!unit_quaternion)
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "the rotation is not a unit quaternion (norm %.6f)", norm);

    ImGui::InputDouble("Chunk length [s]", &config_.chunk_seconds, 1.0, 5.0, "%.1f");
    if (config_.chunk_seconds < 0.0)
        config_.chunk_seconds = 0.0;
    if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Length of one chunk of message log time; 0 reads the whole file as one chunk");

    const bool ready = !config_.lidar_topic.empty() && !config_.imu_topic.empty() && unit_quaternion;
    ImGui::BeginDisabled(!ready);
    const bool open = ImGui::Button("Open");
    ImGui::EndDisabled();
    return open && ready;
}
