#include "nmea.h"
#include <iostream>

namespace hd_mapping::nmea {

double dm_to_dd(const std::string& dm, char direction, bool is_latitude) {
  if (dm.empty()) return 0.0;

  size_t deg_len = is_latitude ? 2 : 3;

  double degrees = std::stod(dm.substr(0, deg_len));
  double minutes = std::stod(dm.substr(deg_len));

  double dd = degrees + minutes / 60.0;
  if (direction == 'S' || direction == 'W') dd *= -1.0;

  return dd;
}

std::tuple<uint64_t, uint64_t, std::string>
BreakLineFromNMEAFile(const std::string &line) {
  std::istringstream iss(line);
  uint64_t timestampLidar;
  uint64_t timestampUnix;
  std::string nmeaSentence;
  iss >> timestampLidar >> timestampUnix;
  char data;
  iss.read(&data, 1);
  std::getline(iss, nmeaSentence);
  return std::make_tuple(timestampLidar, timestampUnix,
                         nmeaSentence);
}

bool validateNMEAChecksum(const std::string &nmea) {
  if (nmea.empty() || nmea[0] != '$') {
    return false;
  }
  auto asterisk = nmea.find('*');
  if (asterisk == std::string::npos || asterisk + 3 > nmea.size())
    return false;
  unsigned char checksum = 0;
  for (size_t i = 1; i < asterisk; ++i) {
    checksum ^= nmea[i];
  }
  unsigned int expected;
  std::istringstream iss(nmea.substr(asterisk + 1, 2));
  iss >> std::hex >> expected;
  return checksum == expected;
}

std::optional<GNRMCData> parseGNRMC(const std::string &nmea) {
  if (nmea.find("$GNRMC") != 0 && nmea.find("$GPRMC") != 0)
    return std::nullopt;

  std::vector<std::string> fields;
  std::stringstream ss(nmea);
  std::string item;
  while (std::getline(ss, item, ',')) {
    fields.push_back(item);
  }
  if (fields.size() < 12)
    return std::nullopt;

  const auto& latStr = fields[3];
  const auto& lonStr = fields[5];
  const char latDir = fields[4].empty() ? 'N' : fields[4][0];
  const char lonDir = fields[6].empty() ? 'E' : fields[6][0];

  GNRMCData data;
  data.time_utc = fields[1];
  data.status = fields[2].empty() ? 'V' : fields[2][0];
  data.latitude = dm_to_dd(latStr, latDir, true);
  data.lat_dir = latDir;
  data.longitude = dm_to_dd(lonStr, lonDir, false);
  data.lon_dir = lonDir;
  data.speed_knots = fields[7].empty() ? 0.0 : std::stod(fields[7]);
  data.track_angle = fields[8].empty() ? 0.0 : std::stod(fields[8]);
  data.date = fields[9];
  data.magnetic_variation = fields[10].empty() ? 0.0 : std::stod(fields[10]);
  data.mag_var_dir = fields[11].empty() ? 'E' : fields[11][0];

  return data;
}

std::optional<GNGGAData> parseGNGGA(const std::string &nmea) {
  if (nmea.find("$GNGGA") != 0 && nmea.find("$GPGGA") != 0)
    return std::nullopt;

  std::vector<std::string> fields;
  std::stringstream ss(nmea);
  std::string item;
  while (std::getline(ss, item, ',')) {
    fields.push_back(item);
  }
  if (fields.size() < 15)
    return std::nullopt;

  const auto& latStr = fields[2];
  const auto& lonStr = fields[4];
  const char latDir = fields[3].empty() ? 'N' : fields[3][0];
  const char lonDir = fields[5].empty() ? 'E' : fields[5][0];


  GNGGAData data;
  data.latitude = dm_to_dd(latStr, latDir, true);
  data.lat_dir = latDir;
  data.longitude = dm_to_dd(lonStr, lonDir, false);
  data.lon_dir = lonDir;
  data.altitude = fields[9].empty() ? 0.0 : std::stod(fields[9]);
  //std::cout << "data.altitude " << data.altitude << std::endl;
  data.hdop = fields[8].empty() ? 0.0 : std::stod(fields[8]);
  data.fix_quality = fields[6].empty() ? 0 : std::stoi(fields[6]);
  data.satellites_tracked = fields[7].empty() ? 0 : std::stoi(fields[7]);
  data.age_of_data =
      fields.size() > 13 && !fields[13].empty() ? std::stod(fields[13]) : -1.0;

  return data;
}

} // namespace hd_mapping::nmea
