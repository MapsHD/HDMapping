#!/usr/bin/env bash
# Packs "Easy Forest HDMapping.app" into a DMG whose window shows the app next to an Applications shortcut, so
# installing is one drag (README: Installation -> Mac).
#
# Usage: make_dmg.sh <path to .app> <output .dmg>
set -euo pipefail

app="${1:?usage: make_dmg.sh <app> <dmg>}"
dmg="${2:?usage: make_dmg.sh <app> <dmg>}"

staging="$(mktemp -d)"
trap 'rm -rf "$staging"' EXIT

# ditto keeps the bundle's symlinks, permissions and signature intact.
ditto "$app" "$staging/$(basename "$app")"
ln -s /Applications "$staging/Applications"

rm -f "$dmg"
hdiutil create -volname "Easy Forest HDMapping" -srcfolder "$staging" -fs HFS+ -format UDZO -ov "$dmg"
hdiutil verify "$dmg"
