#!/bin/bash

# Stop the script if any command fails
set -e

# Define colors for output
GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN}Starting Gazebo simulation...${NC}"
# Launch the Gazebo simulation
gnome-terminal -- bash -c "roslaunch go1_simulation go1_simulation.launch; exec bash"

# Wait for a few seconds to ensure Gazebo is fully started
sleep 5

echo -e "${GREEN}Starting navigation with RViz...${NC}"
# Launch the navigation stack with RViz
gnome-terminal -- bash -c "roslaunch go1_simulation navigate.launch rviz:=true; exec bash"

echo -e "${GREEN}Gazebo and Navigation launch files started successfully!${NC}"
