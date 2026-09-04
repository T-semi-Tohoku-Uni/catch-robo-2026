#!/usr/bin/env bash

set -Eeuo pipefail

readonly INTERFACE_NAME="${1:-vcan_nhk26}"
readonly TEST_VALUE=305419896  # 0x12345678
readonly EXPECTED_CAN_PAYLOAD="12345678"

CREATED_INTERFACE=false
LAUNCH_PID=""
CANDUMP_PID=""
ECHO_PID=""
TEST_TMP_DIR=""
ADMIN_COMMAND=()

log()
{
    printf '[vcan-test] %s\n' "$*"
}

show_launch_log()
{
    if [[ -n "${TEST_TMP_DIR}" && -f "${TEST_TMP_DIR}/launch.log" ]]
    then
        printf '\n--- bridge launch log ---\n' >&2
        tail -n 80 "${TEST_TMP_DIR}/launch.log" >&2 || true
    fi
}

cleanup()
{
    local exit_code=$?
    trap - EXIT INT TERM
    set +e

    for child_pid in "${CANDUMP_PID}" "${ECHO_PID}"
    do
        if [[ -n "${child_pid}" ]] && kill -0 "${child_pid}" 2>/dev/null
        then
            kill -TERM "${child_pid}" 2>/dev/null
            wait "${child_pid}" 2>/dev/null
        fi
    done

    if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null
    then
        kill -INT -- "-${LAUNCH_PID}" 2>/dev/null
        wait "${LAUNCH_PID}" 2>/dev/null
    fi

    if ${CREATED_INTERFACE}
    then
        "${ADMIN_COMMAND[@]}" "${IP_COMMAND}" link delete dev "${INTERFACE_NAME}" \
            >/dev/null 2>&1
    fi

    if [[ -n "${TEST_TMP_DIR}" && -d "${TEST_TMP_DIR}" ]]
    then
        rm -r -- "${TEST_TMP_DIR}"
    fi

    exit "${exit_code}"
}

fail()
{
    printf '[vcan-test] ERROR: %s\n' "$*" >&2
    show_launch_log
    exit 1
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ ! "${INTERFACE_NAME}" =~ ^[[:alnum:]_.-]{1,15}$ ]]
then
    fail "invalid interface name: ${INTERFACE_NAME}"
fi

for required_command in ip ros2 candump cansend setsid
do
    command -v "${required_command}" >/dev/null 2>&1 || \
        fail "required command is missing: ${required_command}"
done

IP_COMMAND="$(command -v ip)"
readonly IP_COMMAND

if [[ ${EUID} -eq 0 ]]
then
    ADMIN_COMMAND=()
else
    command -v sudo >/dev/null 2>&1 || fail "sudo is required to create vcan"
    ADMIN_COMMAND=(sudo -n)
fi

ros2 pkg prefix nhk2026_bridge >/dev/null 2>&1 || \
    fail "nhk2026_bridge is not installed; build the workspace and source install/setup.bash"

if "${IP_COMMAND}" link show dev "${INTERFACE_NAME}" >/dev/null 2>&1
then
    fail "${INTERFACE_NAME} already exists; choose another test-only interface name"
fi

TEST_TMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/nhk2026-vcan-test.XXXXXX")"

if command -v modprobe >/dev/null 2>&1
then
    "${ADMIN_COMMAND[@]}" "$(command -v modprobe)" vcan >/dev/null 2>&1 || true
fi

log "creating ${INTERFACE_NAME} with CAN FD MTU"
if ! "${ADMIN_COMMAND[@]}" "${IP_COMMAND}" link add dev "${INTERFACE_NAME}" type vcan
then
    fail "failed to create ${INTERFACE_NAME}; run 'sudo -v' first or execute as root"
fi
CREATED_INTERFACE=true

"${ADMIN_COMMAND[@]}" "${IP_COMMAND}" link set dev "${INTERFACE_NAME}" mtu 72
"${ADMIN_COMMAND[@]}" "${IP_COMMAND}" link set dev "${INTERFACE_NAME}" up

actual_mtu="$(<"/sys/class/net/${INTERFACE_NAME}/mtu")"
[[ "${actual_mtu}" == "72" ]] || fail "unexpected MTU: ${actual_mtu} (expected 72)"

log "starting nhk2026_canbridge with raspi_canbridge.yml and ifname=${INTERFACE_NAME}"
setsid ros2 launch nhk2026_bridge vcan_can.launch.py \
    "ifname:=${INTERFACE_NAME}" >"${TEST_TMP_DIR}/launch.log" 2>&1 &
LAUNCH_PID=$!

bridge_active=false
for ((attempt = 0; attempt < 75; ++attempt))
do
    if ! kill -0 "${LAUNCH_PID}" 2>/dev/null
    then
        fail "bridge launch exited before becoming active"
    fi

    lifecycle_state="$(ros2 lifecycle get /nhk2026_canbridge 2>/dev/null || true)"
    if [[ "${lifecycle_state}" == *"active [3]"* ]]
    then
        bridge_active=true
        break
    fi
    sleep 0.2
done
[[ "${bridge_active}" == true ]] || fail "bridge did not reach the active lifecycle state"

log "checking ROS -> CAN: /pump_state -> CAN ID 0x401"
candump -L -n 1 -T 5000 "${INTERFACE_NAME},401:C00007FF" \
    >"${TEST_TMP_DIR}/candump.log" 2>&1 &
CANDUMP_PID=$!
sleep 0.2

if ! ros2 topic pub --once --max-wait-time-secs 5 \
    /pump_state std_msgs/msg/Int32MultiArray "{data: [${TEST_VALUE}]}" \
    >"${TEST_TMP_DIR}/topic_pub.log" 2>&1
then
    fail "failed to publish /pump_state"
fi

if ! wait "${CANDUMP_PID}"
then
    CANDUMP_PID=""
    fail "candump failed while waiting for CAN ID 0x401"
fi
CANDUMP_PID=""

grep -Eq "401##[[:xdigit:]]${EXPECTED_CAN_PAYLOAD}" "${TEST_TMP_DIR}/candump.log" || \
    fail "CAN ID 0x401 did not contain the expected big-endian payload"
log "PASS ROS -> CAN (ID 0x401, payload 0x${EXPECTED_CAN_PAYLOAD})"

log "checking CAN -> ROS: CAN ID 0x404 -> /pump"
ros2 topic echo --once --timeout 5 --field data \
    /pump std_msgs/msg/Int32MultiArray >"${TEST_TMP_DIR}/topic_echo.log" 2>&1 &
ECHO_PID=$!

for ((attempt = 0; attempt < 20; ++attempt))
do
    kill -0 "${ECHO_PID}" 2>/dev/null || break
    cansend "${INTERFACE_NAME}" 404##112345678
    sleep 0.2
done

if ! wait "${ECHO_PID}"
then
    ECHO_PID=""
    fail "no /pump message was received for CAN ID 0x404"
fi
ECHO_PID=""

grep -q "${TEST_VALUE}" "${TEST_TMP_DIR}/topic_echo.log" || \
    fail "/pump did not contain the expected value ${TEST_VALUE}"
log "PASS CAN -> ROS (${TEST_VALUE})"
log "all vcan bridge checks passed"
