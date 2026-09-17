#!/usr/bin/env bash
# 중력보상 비교 데모를 띄운다.
set -eo pipefail
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
source ../install/setup.bash
exec .venv/bin/python gravity_demo.py "$@"
