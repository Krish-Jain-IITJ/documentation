#!/bin/bash
# Remove previous build artifacts to avoid stale symlinks
rm -rf build install log

echo "Workspace cleaned: removed build/, install/, and log/ directories."
