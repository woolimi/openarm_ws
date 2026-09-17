#!/usr/bin/env bash
# MuJoCo 데모용 가상환경을 만든다. 한 번만 실행하면 된다.
# ROS 의 xacro·ament_index 를 그대로 쓰려고 시스템 패키지를 물려받는다.
set -euo pipefail
cd "$(dirname "$0")"
python3 -m venv --system-site-packages .venv
.venv/bin/pip install --quiet --upgrade pip
.venv/bin/pip install --quiet mujoco pillow
echo "준비 완료 — ./run.sh 로 실행하세요."
