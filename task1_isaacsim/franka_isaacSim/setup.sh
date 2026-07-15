#!/bin/bash
set -e

# Create assets directory
mkdir -p assets
mkdir -p scripts
mkdir -p extra
mkdir -p docker/isaac-sim/data/runtime_logs/Kit
chmod -R a+rwX docker/isaac-sim/data || true
chmod -R a+rwX extra || true

# Initialize submodules
echo "Initializing submodules..."
git submodule update --init --recursive

# Build Docker images
echo "Building Docker images..."
docker compose -f docker-compose.yml build
