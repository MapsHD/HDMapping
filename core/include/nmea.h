#include <optional>
#include <sstream>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

namespace hd_mapping::nmea {
struct GNRMCData {
  std::string time_utc;
  char status;
  double latitude;
  char lat_dir;
  double longitude;
  char lon_dir;
  double speed_knots;
  double track_angle;
  std::string date;
  double magnetic_variation;
  char mag_var_dir;
};

struct GNGGAData {
  double latitude;
  char lat_dir;
  double longitude;
  char lon_dir;
  double altitude;
  double hdop;
  int fix_quality;
  int satellites_tracked;
  double age_of_data;
};

//! Breaks a line from an NMEA file into its components.
//! Returns a tuple containing the timestamp in nanosecs, the Unix timestamp in
//! nanosecs, and the NMEA sentence.
std::tuple<uint64_t , uint64_t, std::string>
BreakLineFromNMEAFile(const std::string &line);

//! Validates the checksum of an NMEA sentence.
bool validateNMEAChecksum(const std::string &nmea);

//! Parses a GNRMC or GPRMC NMEA sentence and returns a GNRMCData structure.
//! Example sentence: `$GPRMC,184942.40,A,5212.01834,N,02056.49288,E,1.953,254.11,080725,,,R,V*00`
//! If the sentence is invalid or does not contain the required fields, returns
//! std::nullopt.
std::optional<GNRMCData> parseGNRMC(const std::string &nmea);

//! Parses a GNGGA or GPGGA NMEA sentence and returns a GNGGAData structure.
//! Example sentence: `$GPGGA,184852.00,5212.02878,N,02056.51988,E,4,12,0.94,109.1,M,34.4,M,2.0,0093*74`
//! If the sentence is invalid or does not contain the required fields, returns
//! std::nullopt.
std::optional<GNGGAData> parseGNGGA(const std::string &nmea);
} // namespace hd_mapping::nmea
