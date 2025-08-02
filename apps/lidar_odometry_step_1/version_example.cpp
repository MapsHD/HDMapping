// Example: How to use version information from TOML configuration files
// This demonstrates reading and validating version information

#include "toml_io.h"
#include "lidar_odometry_utils.h"
#include <iostream>

void example_version_usage() {
    std::cout << "=== Version Information Usage Example ===" << std::endl;
    
    // 1. Create parameters with current version
    LidarOdometryParams params;
    std::cout << "Current software version: " << params.software_version << std::endl;
    std::cout << "Config version: " << params.config_version << std::endl;
    std::cout << "Build date: " << params.build_date << std::endl;
    
    // 2. Save configuration to TOML file
    TomlIO toml_io;
    std::string config_file = "example_config.toml";
    
    if (toml_io.SaveParametersToTomlFile(config_file, params)) {
        std::cout << "Configuration saved to: " << config_file << std::endl;
    }
    
    // 3. Load and validate version compatibility
    LidarOdometryParams loaded_params;
    if (toml_io.LoadParametersFromTomlFile(config_file, loaded_params)) {
        std::cout << "Configuration loaded successfully!" << std::endl;
        
        // Version compatibility check
        std::string current_version = get_software_version();
        std::string loaded_version = loaded_params.software_version;
        
        if (current_version == loaded_version) {
            std::cout << "✅ Version match: " << current_version << std::endl;
        } else {
            std::cout << "⚠️  Version mismatch:" << std::endl;
            std::cout << "   Current: " << current_version << std::endl;
            std::cout << "   Config:  " << loaded_version << std::endl;
            
            // You can implement migration logic here
            if (should_migrate_config(loaded_version, current_version)) {
                std::cout << "   Migration required!" << std::endl;
            }
        }
        
        // Build date information
        std::cout << "Config was created on: " << loaded_params.build_date << std::endl;
    }
}

// Example version comparison function
bool should_migrate_config(const std::string& old_version, const std::string& new_version) {
    // Parse version numbers (simplified example)
    auto parse_version = [](const std::string& version) {
        std::vector<int> parts;
        std::stringstream ss(version);
        std::string item;
        while (std::getline(ss, item, '.')) {
            parts.push_back(std::stoi(item));
        }
        return parts;
    };
    
    auto old_parts = parse_version(old_version);
    auto new_parts = parse_version(new_version);
    
    // Migration needed if major or minor version changed
    if (old_parts.size() >= 2 && new_parts.size() >= 2) {
        return (old_parts[0] != new_parts[0]) || (old_parts[1] != new_parts[1]);
    }
    
    return false;
}

// Example of reading version from existing TOML without loading full config
void check_config_version(const std::string& config_file) {
    try {
        auto config = toml::parse_file(config_file);
        
        if (config.contains("version_info")) {
            auto version_info = config["version_info"];
            
            if (version_info.contains("software_version")) {
                std::string file_version = version_info["software_version"].value_or("");
                std::string current_version = get_software_version();
                
                std::cout << "Quick version check:" << std::endl;
                std::cout << "  File version: " << file_version << std::endl;
                std::cout << "  Current version: " << current_version << std::endl;
                
                if (file_version == current_version) {
                    std::cout << "  Status: ✅ Compatible" << std::endl;
                } else {
                    std::cout << "  Status: ⚠️  May need update" << std::endl;
                }
            }
        } else {
            std::cout << "⚠️  No version information found in config file!" << std::endl;
            std::cout << "   This config was created with an older version." << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error reading config: " << e.what() << std::endl;
    }
}

/*
Usage in your applications:

1. Always check version compatibility before using config:
   check_config_version("my_config.toml");

2. Handle version mismatches gracefully:
   - Warn user about potential incompatibilities
   - Offer to update config to current version
   - Implement migration logic for breaking changes

3. Version info is automatically saved in TOML files under [version_info]:
   [version_info]
   software_version = "0.84.0"
   config_version = "1.0"
   build_date = "Aug  3 2025"

4. Use build_date to determine config age:
   - Helpful for debugging issues
   - Can suggest updating old configurations
   - Track when settings were last modified
*/
