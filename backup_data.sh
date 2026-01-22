#!/bin/bash

# Configuration
LOCAL_DIR=~/accuracy_data
REMOTE_NAME="gdrive"
REMOTE_FOLDER="accuracy_data"

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "Starting Data Backup..."

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
    echo "Please run the accuracy investigation node first."
    exit 1
fi

# 4. Sync
echo "Syncing '$LOCAL_DIR' to '${REMOTE_NAME}:${REMOTE_FOLDER}/'..."
# Use sync to mirror the folder. Use copy if you want to keep deleted files on remote.
# User asked "synchronisiert werden". sync is usually appropriate, but copy is safer.
# "Ordner soll mit der Cloud synchronisiert werden" -> sync implies mirroring.
rclone sync "$LOCAL_DIR" "${REMOTE_NAME}:${REMOTE_FOLDER}/" --progress

if [ $? -eq 0 ]; then
    echo -e "${GREEN}Success! Data synchronized.${NC}"
else
    echo -e "${RED}Sync failed.${NC}"
    exit 1
fi
