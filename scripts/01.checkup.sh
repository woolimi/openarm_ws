#!/usr/bin/env bash
#
# 01.checkup.sh — OpenArm v1.0 팔로워 점검 (대화형)
#
# 실물 팔을 bringup 하기 전, CAN 버스→모터→영점→관절 한계→관절 구동을 순서대로 점검한다.
# 1·2 는 업스트림 openarm-can-cli, 3·4 는 openarm_follower 의 CLI 를 쓰므로 colcon build
# 가 끝나 있어야 한다. 메뉴에서 절차를 골라 실행한다:
#
#   0) CAN FD 설정            scripts/canup.sh 로 1M/5M CAN-FD      (sudo) ← 먼저
#   1) 팔로워 모터 점검       show_param 으로 모터 1~8 응답 확인      (CAN · 읽기 전용 · 팔별 판정)
#   2) 영점 세팅              현재 자세를 set_zero                    (팔 안 움직임)
#   3) 수동 관절 한계 측정    모터를 끈 채 손으로 스톱→스톱, 하한·상한을 URDF 와 비교 (팔 안 움직임)
#   4) 자동 관절 구동 테스트  URDF 한계 안쪽까지 한 관절씩 천천히, 원점 복귀 (팔 구동)
#
# 2~4 는 bringup(ros2_control)이 꺼져 있어야 한다(CAN 버스 충돌). 3 의 한계는 2 의
# 영점 기준이라 영점을 다시 잡으면 3 을 다시 잰다.
#
# 리더암 점검은 여기 없고 openarm_leader 가 맡는다:
#   ros2 run openarm_leader check
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/.." && pwd)"

# --- 설정 ---------------------------------------------------------------
RIGHT_IFACE=can0                        # 오른팔 CAN 인터페이스
LEFT_IFACE=can1                         # 왼팔 CAN 인터페이스
EXPECT=8                                # 팔 하나의 모터 수 (관절 7 + 그리퍼)
ROS_SETUP=/opt/ros/jazzy/setup.bash
WS_SETUP="$REPO_ROOT/install/setup.bash"
CLI=openarm-can-cli                     # 워크스페이스의 업스트림 openarm_can 이 설치한다
# ------------------------------------------------------------------------
IFACES=("$RIGHT_IFACE" "$LEFT_IFACE")

# ROS + 워크스페이스 소싱 prefix (openarm-can-cli 와 ros2 run 대상이 워크스페이스에서 옴)
SRC="source $ROS_SETUP >/dev/null 2>&1; source $WS_SETUP >/dev/null 2>&1;"

# 0) CAN FD 설정 -------------------------------------------------------------
# canup.sh 가 인터페이스마다 down → bitrate 1M/dbitrate 5M/fd on → up 을 하고,
# 스스로 sudo 로 다시 실행한다.
proc_can_fd() {
    "$HERE/canup.sh" "${IFACES[@]}"
}

# 버스 선점 감지 -------------------------------------------------------------
# bringup(ros2_control)이 돌고 있으면 CLI 질의 응답이 제어 트래픽과 뒤섞여
# 모터가 전부 무응답으로 보인다. 우리는 아무것도 보내지 않으므로, RX+TX 패킷
# 카운터가 저절로 늘면 다른 프로세스가 버스를 쓰는 중이다. (pcan 드라이버는 RX
# 카운터 갱신이 게을러서 TX 도 함께 본다. bringup 은 제어 명령을 상시 송신하므로
# TX 로 확실히 잡힌다.)
_pkts() { echo $(( $(cat "/sys/class/net/$1/statistics/rx_packets") \
                 + $(cat "/sys/class/net/$1/statistics/tx_packets") )); }
# 0 = 이 인터페이스가 점유돼 있음. 카운터를 못 읽으면(인터페이스 부재) 점유가
# 아니라 부재이므로 1 — 질의를 그대로 진행시켜 CLI 가 소켓 에러로 이유를 말하게 둔다.
_iface_busy() {
    local n1 n2
    n1="$(_pkts "$1" 2>/dev/null)" || return 1
    sleep 0.4
    n2="$(_pkts "$1" 2>/dev/null)" || return 1
    (( n2 > n1 ))
}
# 쓰기 절차(영점)는 버스가 하나라도 점유돼 있으면 전체를 멈춘다. 한 팔만 영점을
# 새로 잡고 다른 팔은 못 잡은 어긋난 상태가 아무것도 안 한 것보다 나쁘다.
# 읽기 전용인 모터 점검은 반대로 인터페이스별로 판정한다(proc_motor_check).
_bus_busy() {
    local i
    for i in "${IFACES[@]}"; do
        if _iface_busy "$i"; then
            echo "  $i 에 다른 트래픽이 흐르고 있습니다. bringup(ros2_control) 실행 중으로 보입니다." >&2
            echo "  같은 CAN 버스를 두 프로세스가 쓸 수 없습니다. bringup 을 끄고(Ctrl-C) 다시 실행하세요." >&2
            return 0
        fi
    done
    return 1
}

# 인터페이스 → 팔 이름 (표시용)
_side_of() {
    case "$1" in
        "$RIGHT_IFACE") echo "오른팔" ;;
        "$LEFT_IFACE")  echo "왼팔" ;;
        *)              echo "미지정" ;;
    esac
}

# CAN ID → 사람이 읽는 모터 이름. 배치는 openarm_hardware 의 DEFAULT_SEND_CAN_IDS 를
# 따라 1~7 이 관절(J1~J7), 8 이 그리퍼다. 팔 접두사는 R(오른팔)/L(왼팔)이고 범위 밖
# ID 는 0x 표기 그대로 둔다. 판정·집합 연산은 계속 0x ID 로 하고 표시만 바꾼다.
_motor_label() {
    local side name id=$(($2))
    case "$1" in
        "$RIGHT_IFACE") side="R-" ;;
        "$LEFT_IFACE")  side="L-" ;;
        *)              side="" ;;
    esac
    if   (( id >= 1 && id <= 7 )); then name="J$id"
    elif (( id == 8 ));            then name="그리퍼"
    else                                name="$(printf '0x%x' "$id")"
    fi
    echo "${side}${name}"
}
# 터미널 표시 폭 기준 좌측 패딩. printf 의 %-Ns 는 바이트 폭이라 한글(UTF-8
# 3바이트·표시 2칸)이 섞인 표가 어긋난다. 3바이트 문자를 2칸으로 근사하면
# 한글은 정확하고 ✓✗ 만 1칸 과대인데, 그 둘은 자기 열 안에서 균일해 정렬이
# 유지된다. (grep 의 [가-힣] 범위는 collation 에 따라 실패해 쓰지 않는다.)
_pad() {
    local str="$1" width="$2" bytes chars cols
    bytes="$(printf '%s' "$str" | wc -c)"
    chars="$(printf '%s' "$str" | wc -m)"
    cols=$(( chars + (bytes - chars) / 2 ))
    printf '%s%*s' "$str" "$(( width > cols ? width - cols : 1 ))" ""
}

# 1) 팔로워 모터 점검 (읽기 전용 · show_param) -------------------------------
# show_param 은 현재 버스 보레이트(0 단계가 잡은 5M)에서 모터 파라미터를 읽기만
# 한다(스윕·변경·전원 없음). 모터마다 'MOTOR ID:' 헤더를, 무응답이면
# 'NO RESPONSE FROM MOTOR' 를 찍는다. 종료 코드는 무응답 개수를 반영하지 않고
# (소켓을 못 열 때만 1) 출력도 인터페이스별로 갈려 있지 않으므로, 판정은 이
# 스크립트가 헤더를 세어서 한다.
#
# 팔마다 따로 질의하고 따로 판정한다. 한 팔이 통째로 죽었든 그 버스만 점유돼
# 있든 나머지 팔은 끝까지 본다. 두 팔의 응답 수를 한 숫자로 합치면 정작 알고
# 싶은 것(어느 쪽이 문제인가)이 사라진다.
proc_motor_check() {
    local rc=0 i id side out queried lbl
    local -A probed_of=() missing_of=() note_of=()

    for i in "${IFACES[@]}"; do
        side="$(_side_of "$i")"
        probed_of[$i]=""; missing_of[$i]=""; note_of[$i]=""
        if _iface_busy "$i"; then
            echo "  $i ($side): 건너뜀. 이 버스에 다른 트래픽이 흐릅니다(bringup 실행 중?)." >&2
            note_of[$i]="건너뜀 — 버스 점유"
            rc=1
            continue
        fi

        echo "  $i ($side) 질의 중..."
        out="$(bash -c "$SRC $CLI -i $i show_param" 2>&1)"

        queried="$(grep -c "MOTOR ID:" <<< "$out" || true)"
        if [[ "$queried" -eq 0 ]]; then
            echo "$out"
            echo "  모터 응답 헤더 없음. 0) CAN FD 설정으로 5M 를 먼저 잡으세요." >&2
            note_of[$i]="질의 실패 — 인터페이스 미설정"
            rc=1
            continue
        fi
        if [[ "$queried" -ne "$EXPECT" ]]; then
            echo "  · 질의 대상이 $queried 개로 기대 $EXPECT 개와 다릅니다 (CLI 기본 ID 범위 확인)." >&2
        fi

        probed_of[$i]="$(awk '/MOTOR ID:/{printf "%s ", $3}' <<< "$out")"
        missing_of[$i]="$(awk '/MOTOR ID:/{id=$3} /NO RESPONSE FROM MOTOR/{printf "%s ", id}' <<< "$out")"
        [[ -n "${missing_of[$i]}" ]] && rc=1
    done

    # 종합 판정 — 모터별 응답 여부. 전원 단선도 CAN 단선도 버스에는 똑같이 "없음"
    # 이라 어댑터로는 둘을 가를 수 없다.
    echo
    echo "  ── 종합 판정 ──"
    _row() { echo "  $(_pad "$1" 10)$2"; }
    _row "모터" "응답"
    for i in "${IFACES[@]}"; do
        for id in ${probed_of[$i]:-}; do
            lbl="$(_motor_label "$i" "$id")"
            if [[ " ${missing_of[$i]} " != *" $id "* ]]; then
                _row "$lbl" "✓"
            else
                _row "$lbl" "✗"
            fi
        done
    done
    for i in "${IFACES[@]}"; do
        [[ -n "${note_of[$i]:-}" ]] && echo "  $i ($(_side_of "$i")): ${note_of[$i]}"
    done
    return $rc
}

# 2) 영점 세팅 ---------------------------------------------------------------
# openarm-can-cli set_zero 가 현재 자세를 각 모터의 영점으로 저장한다(팔은 안 움직임).
# 3 은 이 영점 기준으로 재고, 4 는 URDF 한계를 이 영점 기준 모터 각으로 쓴다.
proc_zero_set() {
    local i yn
    _bus_busy && return 1
    echo "  ⚠ 현재 팔 자세를 각 모터의 영점으로 저장합니다 (팔은 안 움직임)."
    echo "    팔을 어깨에서 아래로 곧게 뻗고 그리퍼가 바닥을 향한 영점 자세에 두고,"
    echo "    양쪽 그리퍼는 손으로 끝까지 닫은 뒤 실행하세요. 하드웨어가 닫힌 그리퍼를 0 rad 으로 봅니다."
    echo "    set_zero 동안 모터가 꺼지므로 팔을 받쳐 드세요."
    read -rp "  진행? [y/N]: " yn
    [[ "$yn" == [yY] ]] || { echo "  취소됨"; return 0; }
    for i in "${IFACES[@]}"; do
        echo "  $CLI -i $i set_zero"
        bash -c "$SRC $CLI -i $i set_zero" || return 1
    done
}

# 3·4 공용 — 한 팔을 골라 openarm_follower 의 CLI 를 실행한다 -----------------
# 두 CLI 모두 URDF 관절 한계를 읽고 파일에는 쓰지 않는다. 그 팔의 버스가 비어
# 있을 때만 돈다.
_run_arm_tool() {
    local exe="$1" side iface
    read -rp "  대상 팔 [right/left] (기본 right): " side; side="${side:-right}"
    case "$side" in
        right) iface="$RIGHT_IFACE" ;;
        left)  iface="$LEFT_IFACE" ;;
        *) echo "  right/left 만 가능" >&2; return 1 ;;
    esac
    if _iface_busy "$iface"; then
        echo "  $iface 에 다른 트래픽이 흐르고 있습니다. bringup 을 끄고 다시 실행하세요." >&2
        return 1
    fi
    echo "  ros2 run openarm_follower $exe --arm $side --interface $iface"
    bash -c "$SRC exec ros2 run openarm_follower $exe --arm $side --interface $iface"
}

# 3) 수동 관절 한계 측정 (모터 꺼짐 · 손으로) ----------------------------------
# joint_limit_measure 는 모터에 enable 을 보내지 않고 위치만 읽는다. 관절마다 양 끝
# 스톱까지 손으로 움직이면 지나간 하한·상한을 재어 URDF 한계와 나란히 표로 보여 준다.
proc_limit_measure() { _run_arm_tool joint_limit_measure; }

# 4) 자동 관절 구동 테스트 (팔 구동) --------------------------------------------
# joint_drive_test 가 URDF 관절 한계 안쪽까지 한 관절씩 최고 15°/s(↑·↓ 로 5~30°/s)로
# 움직이고 원점으로 되돌린다. J1·J2 는 한계각의 80%, 나머지는 한계 3° 안쪽까지만 간다.
proc_drive_test() { _run_arm_tool joint_drive_test; }

# 단계 실행기: 실패해도 메뉴로 돌아온다 -------------------------------------
run_step() {
    local label="$1" fn="$2" rc
    echo
    echo "==> $label 시작"
    "$fn"; rc=$?
    if [[ $rc -eq 0 ]]; then
        echo "✓ $label 완료"
    else
        echo "✗ $label 실패 (코드 $rc)" >&2
    fi
    return $rc
}

print_menu() {
    cat <<'MENU'

────────────────────────────────────────────────
 OpenArm 팔로워 점검
   0) CAN FD 설정            (sudo · 먼저 실행)
   1) 팔로워 모터 점검       (CAN · 읽기 전용)
   2) 영점 세팅              (set_zero · 팔 안 움직임 · bringup OFF)
   3) 수동 관절 한계 측정    (모터 꺼짐 · 손으로 스톱→스톱 · bringup OFF)
   4) 자동 관절 구동 테스트  (팔 구동 · 한 관절씩 천천히 · bringup OFF)
   q) 종료
────────────────────────────────────────────────

 리더암 점검은 openarm_leader 에 있습니다:
   ros2 run openarm_leader check
MENU
}

preflight() {
    [[ -f "$ROS_SETUP" ]] || echo "주의: ROS setup 없음 ($ROS_SETUP). ROS 2 Jazzy 설치 필요." >&2
    [[ -f "$WS_SETUP" ]]  || echo "주의: 워크스페이스 미빌드 ($WS_SETUP). colcon build 먼저." >&2
    echo "CAN 인터페이스: $RIGHT_IFACE(오른팔) · $LEFT_IFACE(왼팔)  ·  모터 기대 수: ${EXPECT}/iface"
}

main() {
    preflight
    while true; do
        print_menu
        read -rp "선택 [0/1/2/3/4/q]: " choice
        case "$choice" in
            0) run_step "CAN FD 설정" proc_can_fd ;;
            1) proc_motor_check ;;
            2) run_step "영점 세팅" proc_zero_set ;;
            3) run_step "수동 관절 한계 측정" proc_limit_measure ;;
            4) run_step "자동 관절 구동 테스트" proc_drive_test ;;
            q|Q) echo "종료."; break ;;
            *) echo "0, 1, 2, 3, 4, q 중에서 선택하세요." ;;
        esac
    done
}

main
