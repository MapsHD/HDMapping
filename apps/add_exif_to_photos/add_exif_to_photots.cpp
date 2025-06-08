#include <exiv2/exiv2.hpp>
#include <exiv2/value.hpp>
#include <filesystem>
#include <iostream>
#include <filesystem>
#include <fstream>
Exiv2::Value::UniquePtr makeGpsValue(double value) {
  auto v = Exiv2::Value::create(Exiv2::unsignedRational);

  // Convert to degrees / minutes / seconds
  double absValue = std::abs(value);
  uint32_t degrees = static_cast<uint32_t>(absValue);
  double minutesFull = (absValue - degrees) * 60.0;
  uint32_t minutes = static_cast<uint32_t>(minutesFull);
  double secondsFull = (minutesFull - minutes) * 60.0;
  uint32_t seconds = static_cast<uint32_t>(secondsFull * 10000);  // Exiv2 expects rational

  v->read(std::to_string(degrees) + "/1 " +
          std::to_string(minutes) + "/1 " +
          std::to_string(seconds) + "/10000");

  return v;
}
void SetExifLatLon(const std::string &filename, double latitude, double longitude) {
  auto image = Exiv2::ImageFactory::open(filename);
  image->readMetadata();

  auto &exifData = image->exifData();

  // Latitude ref
  exifData["Exif.GPSInfo.GPSLatitudeRef"] = (latitude >= 0.0) ? "N" : "S";
  exifData["Exif.GPSInfo.GPSLatitude"].setValue(makeGpsValue(latitude).get());

  // Longitude ref
  exifData["Exif.GPSInfo.GPSLongitudeRef"] = (longitude >= 0.0) ? "E" : "W";
  exifData["Exif.GPSInfo.GPSLongitude"].setValue(makeGpsValue(longitude).get());

  // Write back to file
  image->setExifData(exifData);
  image->writeMetadata();
}

std::vector<std::string> getFiles(const std::string &directory, const std::string &extension) {
  std::vector<std::string> imageFiles;
  for (const auto &entry : std::filesystem::directory_iterator(directory)) {
    if (entry.is_regular_file() && (entry.path().extension() == extension)) {
      imageFiles.push_back(entry.path().string());
    }
  }
  std::sort(imageFiles.begin(), imageFiles.end());
  return imageFiles;
}

//extract timestamp from filename photo_278556248895.jpg
uint64_t GetTimestampFromFilename(const std::string &filename) {
  const std::filesystem::path path(filename);
  const std::string stem = path.stem().string();
  const size_t pos = stem.find_last_of('_');
  if (pos != std::string::npos) {
    std::string timestampStr = stem.substr(pos + 1);
    std::stringstream ss(timestampStr);
    uint64_t ts = 0;
    ss >> ts;
    return ts;
  }
  return 0; // or handle error
}

// cache gps entries

void GetGpsDataFromFile(const std::string &gpsFile, std::map<uint64_t, std::pair<double, double>> &gpsCache) {
  std::ifstream file(gpsFile);
  if (!file.is_open()) {
    std::cerr << "Could not open GPS file: " << gpsFile << std::endl;
    return;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    uint64_t timestamp;
    double latitude, longitude;
    if (iss >> timestamp >> latitude >> longitude) {
      gpsCache[timestamp] = {latitude, longitude};
    }
  }
  file.close();
}

int main(int argc, char* argv[]) {

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <directory>" << std::endl;
    return 1;
  }

  const std::string directory = argv[1];

  std::vector<std::string> imageFiles = getFiles(directory, ".jpg");
  std::vector<std::string> gpsFiles = getFiles(directory, ".gnss");
  std::cout << "Found " << imageFiles.size() << " images and " << gpsFiles.size() << " GPS files." << std::endl;

  std::map<uint64_t, std::pair<double, double>> gpsCache;
  for (const auto &gpsFile : gpsFiles) {
    GetGpsDataFromFile(gpsFile, gpsCache);
  }

  std::cout << "Loaded GPS data for " << gpsCache.size() << " timestamps." << std::endl;
  std::cout << "start timestamp: " << double(gpsCache.begin()->first) / 1e9 << std::endl;
  std::cout << "end   timestamp: " << double(gpsCache.rbegin()->first) / 1e9 << std::endl;


  for (const auto &imageFile : imageFiles) {
    uint64_t timestamp = GetTimestampFromFilename(imageFile);
    std::cout << "Processing image: " << imageFile << " with timestamp: " << timestamp << std::endl;

    //find gps data for this timestamp
    auto it = gpsCache.lower_bound(timestamp);
    if (it != gpsCache.end() && it->first == timestamp) {
      // Exact match found
      SetExifLatLon(imageFile, it->second.first, it->second.second);
      std::cout << "Set EXIF for " << imageFile << " with GPS: (" << it->second.first << ", " << it->second.second << ")" << std::endl;
    } else if (it != gpsCache.begin()) {
      // No exact match, use the previous entry
      --it;
      SetExifLatLon(imageFile, it->second.first, it->second.second);
      std::cout << "Set EXIF for " << imageFile << " with GPS: (" << it->second.first << ", " << it->second.second << ") from previous entry." << std::endl;
    } else {
      std::cerr << "No GPS data found for timestamp: " << timestamp << std::endl;
    }
  }

  return 0;
}