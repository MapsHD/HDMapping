// Enhanced example: Robust version handling when loading TOML configurations
// This demonstrates the new version validation and missing version handling

#include "toml_io.h"
#include "lidar_odometry_utils.h"
#include <iostream>

void example_robust_version_handling() {
    std::cout << "=== Robust Version Handling Example ===" << std::endl;
    
    TomlIO toml_io;
    
    // Test scenarios:
    std::vector<std::string> test_configs = {
        "modern_config.toml",    // Has version info
        "legacy_config.toml",    // No version info (older config)
        "incompatible_config.toml" // Different version
    };
    
    for (const auto& config_file : test_configs) {
        std::cout << "\n--- Testing: " << config_file << " ---" << std::endl;
        
        // 1. Quick version check before loading
        auto version_info = toml_io.CheckConfigVersion(config_file);
        
        if (!version_info.found) {
            std::cout << "📄 Legacy config detected (no version info)" << std::endl;
            std::cout << "   → Will auto-update during load" << std::endl;
        } else {
            std::cout << "📄 Config version: " << version_info.software_version << std::endl;
            std::cout << "   Created on: " << version_info.build_date << std::endl;
            
            std::string current = get_software_version();
            if (toml_io.IsVersionCompatible(version_info.software_version, current)) {
                std::cout << "   Status: ✅ Compatible" << std::endl;
            } else {
                std::cout << "   Status: ⚠️  May need update" << std::endl;
            }
        }
        
        // 2. Load configuration (with automatic version handling)
        LidarOdometryParams params;
        if (toml_io.LoadParametersFromTomlFile(config_file, params)) {
            std::cout << "✅ Configuration loaded successfully!" << std::endl;
            std::cout << "   Final version: " << params.software_version << std::endl;
        } else {
            std::cout << "❌ Failed to load configuration" << std::endl;
        }
    }
}

// Example of handling different version scenarios in application code
void application_version_handling_example() {
    std::cout << "\n=== Application Version Handling ===" << std::endl;
    
    TomlIO toml_io;
    std::string config_file = "user_config.toml";
    
    // Check version before deciding how to proceed
    auto version_info = toml_io.CheckConfigVersion(config_file);
    
    if (!version_info.found) {
        std::cout << "🔄 Detected legacy configuration without version info" << std::endl;
        std::cout << "   Options:" << std::endl;
        std::cout << "   1. Load and auto-update (recommended)" << std::endl;
        std::cout << "   2. Create backup before loading" << std::endl;
        std::cout << "   3. Ask user for confirmation" << std::endl;
        
        // Example: Create backup for safety
        std::string backup_file = config_file + ".backup";
        std::cout << "   → Creating backup: " << backup_file << std::endl;
        // std::filesystem::copy_file(config_file, backup_file); // Uncomment in real code
        
    } else {
        std::string current_version = get_software_version();
        if (!toml_io.IsVersionCompatible(version_info.software_version, current_version)) {
            std::cout << "🔄 Version mismatch detected" << std::endl;
            std::cout << "   File version: " << version_info.software_version << std::endl;
            std::cout << "   Current version: " << current_version << std::endl;
            
            // Determine action based on version difference
            if (version_info.software_version < current_version) {
                std::cout << "   → Config is older, will upgrade during load" << std::endl;
            } else {
                std::cout << "   → Config is newer, potential compatibility issues" << std::endl;
                std::cout << "   → Recommend updating HDMapping software" << std::endl;
            }
        }
    }
    
    // Load with automatic handling
    LidarOdometryParams params;
    bool loaded = toml_io.LoadParametersFromTomlFile(config_file, params);
    
    if (loaded) {
        // Check if version was updated and offer to save
        if (!version_info.found || version_info.software_version != params.software_version) {
            std::cout << "💾 Configuration was updated with current version" << std::endl;
            std::cout << "   Save updated config? (y/n): ";
            
            // In real application, get user input
            char response = 'y'; // Simulated user response
            if (response == 'y' || response == 'Y') {
                if (toml_io.SaveParametersToTomlFile(config_file, params)) {
                    std::cout << "   ✅ Updated configuration saved" << std::endl;
                }
            }
        }
    }
}

/*
What happens when version info is missing from TOML:

1. AUTOMATIC DETECTION:
   - CheckConfigVersion() returns VersionInfo with found=false
   - LoadParametersFromTomlFile() automatically detects missing version
   - Console shows warning about legacy config

2. GRACEFUL HANDLING:
   - Parameters load normally with default version values
   - HandleMissingVersion() updates params with current version info
   - User is informed and suggested to save updated config

3. NO ERRORS OR CRASHES:
   - System continues to work with legacy configs
   - Backward compatibility maintained
   - Optional upgrade path provided

4. USER FEEDBACK:
   Console output examples:
   
   For missing version:
   ⚠️  No version information found in config file: legacy_config.toml
      This config was created with an older version of HDMapping.
      → Updated config with current version: 0.84.0
      → Consider saving the config to persist version information.
   
   For version mismatch:
   ⚠️  Version mismatch detected:
      Config version: 0.83.0
      Current version: 0.84.0
      Config created on: Jul 15 2025
      Consider updating the configuration file.
      
   For compatible version:
   ✅ Version compatible: 0.84.0

BEST PRACTICES:
- Always call CheckConfigVersion() before loading for user feedback
- Use LoadParametersFromTomlFile() which handles everything automatically
- Offer to save updated configs to persist version information
- Create backups when upgrading legacy configs
- Handle version mismatches gracefully with user guidance
*/
