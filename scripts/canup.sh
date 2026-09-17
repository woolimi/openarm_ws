#!/usr/bin/env bash
# OpenArm 팔로워가 쓰는 CAN-FD 인터페이스를 올린다.
# 기본은 can0(오른팔)·can1(왼팔) 이고, 인자로 인터페이스를 주면 그것만 다룬다.
set -euo pipefail

# --- 설정 ---------------------------------------------------------------
BITRATE=1000000          # arbitration phase 비트레이트
DBITRATE=5000000         # data phase 비트레이트
RESTART_MS=100           # bus-off 자동 복구 간격. 0 이면 수동 복구
TXQUEUELEN=1000          # 송신 큐 길이. 짧으면 ENOBUFS 로 전송이 막힌다
DEFAULT_IFACES=(can0 can1)
# ------------------------------------------------------------------------

usage() {
  cat <<USAGE
사용법: canup.sh [옵션] [인터페이스...]

옵션:
  -b, --bitrate <bps>    arbitration 비트레이트 (기본 ${BITRATE})
  -d, --dbitrate <bps>   data 비트레이트 (기본 ${DBITRATE})
  -D, --down             인터페이스를 올리지 않고 내린다
  -h, --help             이 도움말

인터페이스를 생략하면 ${DEFAULT_IFACES[*]} 중 존재하는 것을 모두 올린다.

예시:
  canup.sh                 # can0·can1 을 1M/5M CAN-FD 로 올린다
  canup.sh can1            # 왼팔만 올린다
  canup.sh -d 2000000 can0 # data 비트레이트를 2M 로 올린다
  canup.sh --down          # 둘 다 내린다
USAGE
}

down_only=false
ifaces=()
argv=("$@")   # sudo 재실행 때 원래 인자를 그대로 넘기려고 파싱 전에 보관한다

while (($#)); do
  case "$1" in
    -b|--bitrate)  BITRATE=$2; shift 2 ;;
    -d|--dbitrate) DBITRATE=$2; shift 2 ;;
    -D|--down)     down_only=true; shift ;;
    -h|--help)     usage; exit 0 ;;
    -*)            echo "알 수 없는 옵션: $1" >&2; usage >&2; exit 2 ;;
    *)             ifaces+=("$1"); shift ;;
  esac
done

# ip link 설정은 root 권한이 필요하다.
if ((EUID != 0)); then
  exec sudo -- "$0" "${argv[@]}"
fi

exists() { ip link show "$1" &>/dev/null; }

if ((${#ifaces[@]} == 0)); then
  for i in "${DEFAULT_IFACES[@]}"; do
    exists "$i" && ifaces+=("$i")
  done
  if ((${#ifaces[@]} == 0)); then
    echo "CAN 인터페이스가 없다 (${DEFAULT_IFACES[*]}). pcan 모듈과 어댑터 연결을 확인한다." >&2
    echo "  lsmod | grep pcan" >&2
    exit 1
  fi
else
  for i in "${ifaces[@]}"; do
    if ! exists "$i"; then
      echo "$i 가 없다. pcan 모듈과 어댑터 연결을 확인한다." >&2
      exit 1
    fi
  done
fi

for i in "${ifaces[@]}"; do
  # 비트레이트는 인터페이스가 내려가 있어야 바꿀 수 있다.
  ip link set "$i" down
  $down_only && { echo "$i down"; continue; }

  ip link set "$i" type can \
    bitrate "$BITRATE" \
    dbitrate "$DBITRATE" \
    fd on \
    restart-ms "$RESTART_MS"
  ip link set "$i" txqueuelen "$TXQUEUELEN"
  ip link set "$i" up
  echo "$i up  bitrate ${BITRATE} dbitrate ${DBITRATE} fd on"
done

$down_only || ip -details -brief link show type can
