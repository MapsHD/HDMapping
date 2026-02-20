#!/bin/bash

# Script to create a macOS DMG package for HD Mapping directory bundle
# This script packages the HDMapping directory (not .app bundle) into a distributable .dmg
# Prerequisites: build/HDMapping directory must already exist (created by create_dir_bundle.sh)

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=====================================${NC}"
echo -e "${BLUE}HD Mapping - DMG Creation (Directory Bundle)${NC}"
echo -e "${BLUE}=====================================${NC}"
echo ""

# Configuration
BUNDLE_NAME="HDMapping"
BUILD_DIR="build"
DMG_NAME="${BUILD_DIR}/HDMapping.dmg"
DIR_BUNDLE="${BUILD_DIR}/${BUNDLE_NAME}"

# Verify directory bundle exists
echo -e "${YELLOW}Verifying directory bundle...${NC}"
if [ ! -d "$DIR_BUNDLE" ]; then
    echo -e "${RED}Error: Directory bundle not found at $DIR_BUNDLE${NC}"
    echo -e "${YELLOW}Please run ./macos_bundle_toolkit/create_dir_bundle.sh first${NC}"
    exit 1
fi

# Check if required subdirectories exist
if [ ! -d "$DIR_BUNDLE/bin" ] || [ ! -d "$DIR_BUNDLE/lib" ]; then
    echo -e "${RED}Error: Directory bundle is incomplete (missing bin/ or lib/)${NC}"
    echo -e "${YELLOW}Please run ./macos_bundle_toolkit/create_dir_bundle.sh first${NC}"
    exit 1
fi

echo -e "${GREEN}✓ Directory bundle found at $DIR_BUNDLE${NC}"
echo -e "${GREEN}✓ Contains: $(ls $DIR_BUNDLE/bin | wc -l | xargs) executables, $(ls $DIR_BUNDLE/lib/*.dylib 2>/dev/null | wc -l | xargs) libraries${NC}"
echo ""

# Create DMG
echo -e "${YELLOW}Creating DMG package...${NC}"

# Remove old DMG if exists
rm -f "$DMG_NAME"

# Check if create-dmg is available
if ! command -v create-dmg &> /dev/null; then
    echo -e "${RED}Error: create-dmg not found. Install it with: brew install create-dmg${NC}"
    exit 1
fi

# Create temporary directory for DMG source
TEMP_DMG_DIR=$(mktemp -d)
echo "  Creating temporary directory: $TEMP_DMG_DIR"

# Copy the bundle to temp directory to create proper structure
# This ensures the DMG contains a folder named "HDMapping" that can be dragged to Applications
cp -R "$DIR_BUNDLE" "$TEMP_DMG_DIR/${BUNDLE_NAME}"

# Create DMG with directory bundle
# Note: For a directory (not .app), we don't use --volicon or --hide-extension
# Try with icon positioning first, if that fails, create without icon positioning
create-dmg \
  --volname "HDMapping" \
  --window-pos 200 120 \
  --window-size 800 400 \
  --icon-size 100 \
  --icon "${BUNDLE_NAME}" 200 190 \
  --app-drop-link 600 185 \
  "$DMG_NAME" \
  "$TEMP_DMG_DIR" 2>&1 || \
create-dmg \
  --volname "HDMapping" \
  --window-pos 200 120 \
  --window-size 800 400 \
  --icon-size 100 \
  --app-drop-link 600 185 \
  "$DMG_NAME" \
  "$TEMP_DMG_DIR"

# Clean up temporary directory
rm -rf "$TEMP_DMG_DIR"
echo "  ✓ Cleaned up temporary directory"

# Verify DMG was created
if [ ! -f "$DMG_NAME" ]; then
    echo -e "${RED}Error: DMG file was not created${NC}"
    exit 1
fi

echo -e "${GREEN}✓ DMG created successfully${NC}"
echo ""

# Show summary
echo -e "${BLUE}=====================================${NC}"
echo -e "${BLUE}DMG Package Created Successfully!${NC}"
echo -e "${BLUE}=====================================${NC}"
echo ""
echo "DMG file: $DMG_NAME"
echo "Size: $(du -h "$DMG_NAME" | awk '{print $1}')"
echo "Location: $(pwd)/$DMG_NAME"
echo ""
echo "To test the DMG:"
echo "  open $DMG_NAME"
echo ""
echo "To install:"
echo "  1. Mount the DMG by double-clicking"
echo "  2. Drag ${BUNDLE_NAME} folder to Applications"
echo "  3. Run tools from /Applications/${BUNDLE_NAME}/bin/"
echo ""
echo "Example usage after installation:"
echo "  /Applications/${BUNDLE_NAME}/bin/hd_mapping_launcher_gui"
echo ""
