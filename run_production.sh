#!/bin/sh
# Re-execute in Bash to use the relocatable ROS setup.bash scripts.
if [ -z "${BASH_VERSION:-}" ]; then
    exec /bin/bash "$0" "$@"
fi
set -eo pipefail

robot_workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

fail() {
    printf '起動エラー: %s\n' "$*" >&2
    exit 1
}

if [[ "${1:-}" == '--help' || "${1:-}" == '-h' ]]; then
    cat <<'USAGE'
Usage: ./run_production.sh team:=red [debug:=true] [launch arguments...]
       ROBOT_ROS_DISTRO=humble ./run_production.sh team:=blue

ROS環境とこのワークスペースのinstallを読み直し、CAN付き本番launchを起動します。
ROS選択順: ROBOT_ROS_DISTRO → ROS_DISTRO → /opt/ros内の唯一の環境。
--show-args でlaunch引数を表示できます。ビルドは自動実行しません。
手首実測角の許容設定は joint_feedback_config:=/path/to/joint_feedback.yaml で変更できます。
USAGE
    exit 0
fi

robot_distro=${ROBOT_ROS_DISTRO:-${ROS_DISTRO:-}}
if [[ -z "$robot_distro" ]]; then
    robot_setups=(/opt/ros/*/setup.bash)
    if [[ ${#robot_setups[@]} != 1 || ! -f "${robot_setups[0]}" ]]; then
        fail 'ROBOT_ROS_DISTRO=humble または jazzy などでROS環境を指定してください。'
    fi
    robot_distro=$(basename -- "$(dirname -- "${robot_setups[0]}")")
fi
[[ "$robot_distro" =~ ^[a-z][a-z0-9_]*$ ]] || fail 'ROSディストリビューション名が不正です。'
robot_ros_setup="/opt/ros/$robot_distro/setup.bash"
robot_local_setup="$robot_workspace/install/local_setup.bash"
[[ -f "$robot_ros_setup" ]] || fail "$robot_ros_setup がありません。"
[[ -f "$robot_local_setup" ]] || fail 'installがありません。対象ROS環境でcolcon buildを実行してください。'

# Reject incomplete install trees before sourcing can leave a partial environment.
/usr/bin/python3 - "$robot_local_setup" <<'PY'
from pathlib import Path
import re
import sys

setup = Path(sys.argv[1])
helpers = set(re.findall(r'_local_setup_util[\w]*\.py', setup.read_text()))
missing = [name for name in helpers if not (setup.parent / name).is_file()]
if not helpers or missing:
    print('起動エラー: installの環境読込補助ファイルが欠損しています: ' +
          ', '.join(missing or ['_local_setup_util*.py']), file=sys.stderr)
    print('対象ROS環境でcolcon buildを実行し、installを再生成してください。', file=sys.stderr)
    sys.exit(1)
PY

# Start from system paths; preserve DDS, network and device settings.
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
unset COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX COLCON_PYTHON_EXECUTABLE
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION
source "$robot_ros_setup"
# Avoid stale underlay paths recorded in install/setup.bash at build time.
source "$robot_local_setup"

for robot_package in catchrobo2026_sequence catchrobo2026_ui nav_director nhk2026_bridge; do
    robot_prefix=$(ros2 pkg prefix "$robot_package") || fail "$robot_package が見つかりません。colcon buildを実行してください。"
    if [[ "$robot_prefix" != "$robot_workspace/install" &&
          "$robot_prefix" != "$robot_workspace/install/"* ]]; then
        fail "$robot_package が別のワークスペースを参照しています: $robot_prefix"
    fi
done

cd -- "$robot_workspace"
printf 'ROS=%s workspace=%s\n' "$ROS_DISTRO" "$robot_workspace"
exec ros2 launch "$robot_workspace/launch/production.launch.py" "$@"
