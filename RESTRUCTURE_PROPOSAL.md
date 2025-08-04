# HDMapping Project Restructure Proposal

## Executive Summary
This PR proposes a structural reorganization of the HDMapping project to improve maintainability, discoverability, and developer experience without affecting the cross-platform functionality.

## Current Status Analysis
✅ **Cross-Platform Support**: Excellent (Windows/Linux/macOS/ARM)  
✅ **CPU Optimizations**: Auto-detection with SIMD support  
✅ **Build System**: Modern CMake with proper dependency management  
✅ **Testing Infrastructure**: Recently added with CTest integration  
✅ **Documentation**: Well-organized in `docs/` directory  

## Proposed Changes

### 1. Application Categorization
**Problem**: 15+ applications in flat `apps/` structure making it hard to navigate  
**Solution**: Organize by functionality

```
apps/
├── viewers/                    # Data visualization applications
│   ├── mandeye_raw_data_viewer/
│   ├── mandeye_single_session_viewer/
│   └── README.md              # Overview of viewer applications
├── processing/                 # Core data processing
│   ├── lidar_odometry_step_1/
│   ├── multi_session_registration/
│   ├── multi_view_tls_registration/
│   ├── split_multi_livox/
│   └── README.md              # Processing pipeline overview
├── calibration/               # Calibration and intrinsic tools
│   ├── livox_mid_360_intrinsic_calibration/
│   ├── mandeye_mission_recorder_calibration/
│   └── README.md              # Calibration workflow guide
├── tools/                     # Utility and analysis tools
│   ├── compare_trajectories/
│   ├── manual_color/
│   ├── single_session_manual_coloring/
│   ├── precision_forestry_tools/
│   └── README.md              # Tools usage examples
├── demos/                     # Demo and example applications
│   ├── quick_start_demo/
│   ├── hd_mapper/
│   ├── matrix_mul/            # Example/benchmark
│   └── README.md              # Getting started guide
└── README.md                  # Complete application overview
```

### 2. External Tools Organization
**Problem**: Scripts scattered in root directory  
**Solution**: Centralized tools directory

```
tools/                         # External utilities (moved from root)
├── python/                    # Python scripts (from python_scripts/)
│   ├── pas1.py
│   ├── plot-hist-func.py
│   ├── plotRMS.py
│   └── README.md
├── nmea_converter/            # NMEA tools (from nmea_to_kml_converter/)
│   ├── instruction_nmea_to_kml.pdf
│   ├── nmea_to_kml_GGA.py
│   ├── nmea_to_kml_GNRMC.py
│   └── README.md
└── README.md                  # Tools overview
```

### 3. CMake Modularization
**Problem**: Monolithic CMakeLists.txt (329 lines)  
**Solution**: Organized cmake modules

```
cmake/
├── modules/                   # Find modules and utilities
│   ├── FindLASzip.cmake      # (existing)
│   └── HDMappingUtils.cmake  # Common functions
├── compiler/                 # Compiler-specific settings
│   ├── cpu_optimization.cmake
│   ├── simd_detection.cmake
│   └── platform_flags.cmake
├── dependencies/             # Third-party management
│   ├── eigen.cmake
│   ├── freeglut.cmake
│   └── laszip.cmake
└── packaging/                # CPack configuration
    └── cpack_config.cmake
```

### 4. Development Infrastructure
**Problem**: No standardized development environment  
**Solution**: Add development tools

```
.vscode/                      # VS Code workspace configuration
├── settings.json            # Project-specific settings
├── tasks.json               # Build and test tasks
├── launch.json              # Debug configurations
└── extensions.json          # Recommended extensions

.github/                     # GitHub workflows and templates
├── workflows/
│   ├── ci.yml              # Continuous integration
│   ├── build-test.yml      # Multi-platform build testing
│   └── documentation.yml   # Auto-generate docs
├── ISSUE_TEMPLATE/
│   ├── bug_report.md
│   ├── feature_request.md
│   └── build_issue.md
└── pull_request_template.md
```

### 5. Enhanced Documentation Structure
**Problem**: Mixed documentation levels  
**Solution**: Hierarchical documentation

```
docs/
├── user/                    # End-user documentation
│   ├── getting_started.md
│   ├── application_guide.md
│   └── troubleshooting.md
├── developer/              # Developer documentation
│   ├── build_system.md
│   ├── contributing.md
│   ├── testing_guide.md
│   └── architecture.md
├── api/                    # API documentation
│   ├── core_api.md
│   └── plugins_api.md
└── examples/              # Usage examples and tutorials
    ├── basic_workflow.md
    ├── advanced_calibration.md
    └── custom_processing.md
```

## Implementation Strategy

### Phase 1: Application Reorganization (Week 1)
1. Create category directories in `apps/`
2. Move applications to appropriate categories
3. Update CMakeLists.txt paths
4. Create category README files
5. Test cross-platform builds

### Phase 2: Tools Consolidation (Week 1)
1. Create `tools/` directory
2. Move `python_scripts/` → `tools/python/`
3. Move `nmea_to_kml_converter/` → `tools/nmea_converter/`
4. Update documentation references
5. Create tools README files

### Phase 3: CMake Modularization (Week 2)
1. Extract CPU optimization logic to separate files
2. Modularize dependency management
3. Create utility functions for common patterns
4. Maintain backward compatibility
5. Test all build configurations

### Phase 4: Development Infrastructure (Week 2)
1. Add VS Code workspace configuration
2. Create GitHub workflow templates
3. Add issue templates
4. Create contribution guidelines
5. Setup automated testing

## Benefits

### For Developers
- **Faster Navigation**: Clear application categorization
- **Easier Onboarding**: Structured documentation and examples
- **Consistent Development**: Standardized VS Code setup
- **Better Testing**: Automated CI/CD workflows

### For Users
- **Clearer Documentation**: Hierarchical help system
- **Easier Discovery**: Categorized applications with guides
- **Better Examples**: Organized tutorials and workflows
- **Faster Troubleshooting**: Structured problem-solving guides

### For Maintainers
- **Modular CMake**: Easier to maintain build system
- **Organized Issues**: Template-driven issue reporting
- **Automated Testing**: Multi-platform CI validation
- **Scalable Structure**: Easy to add new applications/tools

## Risk Mitigation

### Backward Compatibility
- All existing CMake variables and targets preserved
- Build commands remain unchanged
- API compatibility maintained
- Documentation updated with migration notes

### Testing Strategy
- Test all platforms (Windows/Linux/macOS/ARM)
- Validate all CPU optimization modes
- Verify all applications build and run
- Check packaging (DEB/ZIP) generation

## Timeline
- **Week 1**: Applications + Tools reorganization
- **Week 2**: CMake + Development infrastructure
- **Week 3**: Documentation + Testing
- **Week 4**: Final validation + PR submission

## Success Metrics
- [ ] All 15+ applications build successfully on all platforms
- [ ] CMake configuration time reduced by 20%
- [ ] Documentation coverage increased to 90%
- [ ] Developer onboarding time reduced by 50%
- [ ] CI/CD pipeline covers all build configurations

## Conclusion
This restructure maintains HDMapping's excellent cross-platform support while significantly improving project organization and developer experience. The modular approach ensures scalability for future features and easier maintenance.
