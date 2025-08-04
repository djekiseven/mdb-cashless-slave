#!/bin/bash

# Script to switch configuration to ESP32 WROVER
echo "Switching to ESP32 WROVER configuration..."

# Backup original sdkconfig if it doesn't exist
if [ ! -f sdkconfig.s3 ]; then
  echo "Backing up original ESP32-S3 configuration..."
  cp sdkconfig sdkconfig.s3
fi

# Copy WROVER configuration
echo "Applying ESP32 WROVER configuration..."
cp sdkconfig.wrover sdkconfig

echo "Configuration switched to ESP32 WROVER"
echo "Please rebuild your project"
