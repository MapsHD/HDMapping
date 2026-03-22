HDMapping Toolset for macOS
============================

This directory contains the complete HDMapping toolset with all required
dependencies bundled together.

INSTALLATION
------------

Option 1: Copy to Applications (recommended)
  1. Copy the entire HDMapping folder to /Applications:
     cp -r HDMapping /Applications/
  
  2. Add to PATH (optional, for easy command-line access):
     echo 'export PATH="/Applications/HDMapping/bin:$PATH"' >> ~/.zshrc
     source ~/.zshrc

Option 2: Keep in any location
  Simply copy the HDMapping folder wherever you like. All dependencies
  are bundled, so it will work from any location.

USAGE
-----

Run tools directly:
  ./HDMapping/bin/tool_name

Or if added to PATH:
  tool_name

Available tools are located in the bin/ directory. Use:
  ls ./HDMapping/bin/

To see all available commands.

STRUCTURE
---------

HDMapping/
  ├── bin/          All executable tools
  ├── lib/          Bundled dynamic libraries (dependencies)
  └── README.txt    This file

All required libraries are in the lib/ directory and are automatically
found by the executables using relative paths.

TROUBLESHOOTING
---------------

If you encounter "command not found" errors:
  - Make sure you're using the full path to the executable
  - Or add HDMapping/bin to your PATH as described above

If you encounter "library not found" errors:
  - Ensure the lib/ directory is in the same parent folder as bin/
  - Do not separate bin/ and lib/ directories

For more information, visit: https://github.com/MapsHD/HDMapping
