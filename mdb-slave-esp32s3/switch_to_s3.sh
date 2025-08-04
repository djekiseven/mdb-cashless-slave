#!/bin/bash

# Script to switch configuration back to ESP32-S3
echo "Switching to ESP32-S3 configuration..."

# Check if backup exists
if [ -f sdkconfig.s3 ]; then
  echo "Restoring original ESP32-S3 configuration..."
  cp sdkconfig.s3 sdkconfig
else
  echo "Error: ESP32-S3 configuration backup not found!"
  exit 1
fi

echo "Configuration switched to ESP32-S3"
echo "Please rebuild your project"
