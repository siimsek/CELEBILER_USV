#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
gz sim -r shapes.sdf
