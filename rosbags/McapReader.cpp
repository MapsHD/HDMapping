// No MCAP_IMPLEMENTATION define here: McapWriter.cpp already provides the mcap
// library's single implementation, and every target that compiles McapReader.cpp
// also compiles McapWriter.cpp (see apps/console_tools/CMakeLists.txt and
// rosbags/tests/CMakeLists.txt). Defining it in both would be a duplicate-symbol
// link error.
#include "McapReader.h"
#include "cdr_serializer.hpp"

#include <mcap/reader.hpp>

#include <algorithm>
#include <iostream>
#include <unordered_map>

namespace rosbags
{

namespace
{

// ros2 schema names are "sensor_msgs/msg/PointCloud2"; ros1-style bags spell the
// same type "sensor_msgs/PointCloud2". Comparing only the trailing type name
// accepts both.
std::string_view schemaTypeName(std::string_view schema_name)
{
	const auto slash = schema_name.rfind('/');
	return slash == std::string_view::npos ? schema_name : schema_name.substr(slash + 1);
}

} // namespace

struct McapFileReader::Impl
{
	mcap::McapReader reader;
	McapReaderOptions options;
	ResolvedTopics resolved;
	std::vector<ChannelInfo> channels;
	std::string error;
	std::string lidarLayoutWarning;
	bool open = false;
	bool timeOrdered = false;

	// Resolves one stream to a topic name. `requested` non-empty means "use this
	// topic, and fail if it isn't in the file"; empty means auto-detect the
	// single channel carrying `type_name`. `required_unique` streams error out on
	// ambiguity, optional ones give up quietly.
	bool resolveTopic(const std::string& requested, std::string_view type_name, bool required_unique, std::string& out)
	{
		if(!requested.empty())
		{
			const auto it = std::find_if(channels.begin(), channels.end(), [&](const ChannelInfo& c) { return c.topic == requested; });
			if(it == channels.end())
			{
				error = "topic '" + requested + "' is not present in the file";
				return false;
			}
			if(it->message_encoding != "cdr")
			{
				error = "topic '" + requested + "' uses message encoding '" + it->message_encoding + "', expected 'cdr'";
				return false;
			}
			out = requested;
			return true;
		}

		std::vector<const ChannelInfo*> candidates;
		for(const auto& c : channels)
		{
			if(schemaTypeName(c.schema_name) == type_name && c.message_encoding == "cdr")
				candidates.push_back(&c);
		}

		if(candidates.empty())
			return true; // absent stream, `out` stays empty
		if(candidates.size() > 1)
		{
			if(!required_unique)
				return true;
			error = "found " + std::to_string(candidates.size()) + " " + std::string(type_name) + " topics (";
			for(size_t i = 0; i < candidates.size(); ++i)
				error += (i ? ", " : "") + candidates[i]->topic;
			error += ") - pick one explicitly";
			return false;
		}
		out = candidates.front()->topic;
		return true;
	}
};

McapFileReader::McapFileReader(const std::filesystem::path& path, const McapReaderOptions& options)
	: impl_(std::make_unique<Impl>())
{
	impl_->options = options;

	auto status = impl_->reader.open(path.string());
	if(!status.ok())
	{
		impl_->error = "failed to open " + path.string() + ": " + status.message;
		return;
	}

	// AllowFallbackScan so files written without a summary section (or truncated
	// mid-recording) still enumerate their channels.
	status = impl_->reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
	if(!status.ok())
	{
		impl_->error = "failed to read summary of " + path.string() + ": " + status.message;
		return;
	}

	const auto& schemas = impl_->reader.schemas();
	const auto& statistics = impl_->reader.statistics();
	for(const auto& [id, channel] : impl_->reader.channels())
	{
		ChannelInfo info;
		info.topic = channel->topic;
		info.message_encoding = channel->messageEncoding;
		if(const auto it = schemas.find(channel->schemaId); it != schemas.end())
			info.schema_name = it->second->name;
		if(statistics)
		{
			if(const auto it = statistics->channelMessageCounts.find(id); it != statistics->channelMessageCounts.end())
				info.message_count = it->second;
		}
		impl_->channels.push_back(std::move(info));
	}
	std::sort(impl_->channels.begin(), impl_->channels.end(), [](const ChannelInfo& a, const ChannelInfo& b) { return a.topic < b.topic; });

	if(!impl_->resolveTopic(options.lidar_topic, "PointCloud2", /*required_unique=*/true, impl_->resolved.lidar) ||
	   !impl_->resolveTopic(options.imu_topic, "Imu", /*required_unique=*/true, impl_->resolved.imu) ||
	   !impl_->resolveTopic(options.sn_topic, "String", /*required_unique=*/false, impl_->resolved.sn))
		return;

	// Ascending-log-time iteration needs per-chunk message indexes; this mirrors
	// the precondition mcap's own IndexedMessageReader checks, so testing it here
	// lets us fall back to file order instead of failing mid-iteration.
	const auto& chunkIndexes = impl_->reader.chunkIndexes();
	impl_->timeOrdered = !chunkIndexes.empty() && chunkIndexes.front().messageIndexLength != 0;

	impl_->open = true;
}

McapFileReader::~McapFileReader()
{
	if(impl_)
		impl_->reader.close();
}

bool McapFileReader::isOpen() const
{
	return impl_ && impl_->open;
}

const std::string& McapFileReader::error() const
{
	return impl_->error;
}

const ResolvedTopics& McapFileReader::topics() const
{
	return impl_->resolved;
}

const std::vector<ChannelInfo>& McapFileReader::channels() const
{
	return impl_->channels;
}

bool McapFileReader::timeOrdered() const
{
	return impl_ && impl_->timeOrdered;
}

const std::string& McapFileReader::lidarLayoutWarning() const
{
	return impl_->lidarLayoutWarning;
}

bool McapFileReader::read(const Callbacks& callbacks)
{
	if(!isOpen())
		return false;

	const std::string& lidarTopic = impl_->resolved.lidar;
	const std::string& imuTopic = impl_->resolved.imu;
	const std::string& snTopic = impl_->resolved.sn;

	const bool wantLidar = callbacks.onPointCloud && !lidarTopic.empty();
	const bool wantImu = callbacks.onImu && !imuTopic.empty();
	const bool wantSn = callbacks.onSn && !snTopic.empty();
	if(!wantLidar && !wantImu && !wantSn)
		return true;

	mcap::ReadMessageOptions read_options;
	read_options.readOrder = impl_->timeOrdered ? mcap::ReadMessageOptions::ReadOrder::LogTimeOrder
											   : mcap::ReadMessageOptions::ReadOrder::FileOrder;
	read_options.topicFilter = [&](std::string_view topic) {
		return (wantLidar && topic == lidarTopic) || (wantImu && topic == imuTopic) || (wantSn && topic == snTopic);
	};

	bool failed = false;
	const auto onProblem = [&](const mcap::Status& status) {
		if(impl_->error.empty())
			impl_->error = status.message;
		failed = true;
	};

	auto messages = impl_->reader.readMessages(onProblem, read_options);
	for(const auto& view : messages)
	{
		const auto* data = reinterpret_cast<const uint8_t*>(view.message.data);
		const size_t size = view.message.dataSize;
		const std::string& topic = view.channel->topic;

		if(wantLidar && topic == lidarTopic)
		{
			// Only the first complaint is kept: a layout problem is a property of
			// the file, so it would otherwise repeat once per message.
			auto points = decodePc2(data, size, impl_->options.lidar_preset, &impl_->lidarLayoutWarning);
			if(!points.empty())
				callbacks.onPointCloud(view.message.logTime, std::move(points));
		}
		else if(wantImu && topic == imuTopic)
		{
			if(const auto sample = decodeImu(data, size))
				callbacks.onImu(*sample);
		}
		else if(wantSn && topic == snTopic)
		{
			callbacks.onSn(view.message.logTime, decodeSn(data, size));
		}
	}

	return !failed;
}

} // namespace rosbags
