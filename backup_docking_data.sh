#!/bin/bash

# Configuration
LOCAL_DIR=~/docking_test_data
REMOTE_NAME="gdrive"
REMOTE_FOLDER="docking_test_data"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "Starting Docking Test Suite Data Backup..."

# 1. Check if rclone is installed
if ! command -v rclone &> /dev/null; then
    echo -e "${RED}Error: rclone is not installed.${NC}"
    echo "Please install it using: sudo apt install rclone"
    exit 1
fi

# 2. Check if remote is configured
if ! rclone listremotes | grep -q "${REMOTE_NAME}:"; then
    echo -e "${RED}Error: Remote '${REMOTE_NAME}' is not configured.${NC}"
    echo "Please configure it using: rclone config"
    echo "  - Create a new remote named '${REMOTE_NAME}'"
    echo "  - Select 'drive' (Google Drive)"
    exit 1
fi

# 3. Check if source directory exists
if [ ! -d "$LOCAL_DIR" ]; then
    echo -e "${RED}Error: Source directory '$LOCAL_DIR' not found.${NC}"
    echo "Please run the docking test suite first."
    exit 1
fi

# 4. Sync
echo "Syncing '$LOCAL_DIR' to '${REMOTE_NAME}:${REMOTE_FOLDER}/'..."
# Use sync to mirror the folder.
rclone sync "$LOCAL_DIR" "${REMOTE_NAME}:${REMOTE_FOLDER}/" --progress

if [ $? -eq 0 ]; then
    echo -e "${GREEN}Success! Data synchronized.${NC}"
else
    echo -e "${RED}Sync failed.${NC}"
    exit 1
fi
