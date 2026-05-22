#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-$PWD/install/setup.bash}"
PX4_TARGET="${PX4_TARGET:-gz_x500_depth}"
XRCE_PORT="${XRCE_PORT:-8888}"
AUTONOMY_LAUNCH="${AUTONOMY_LAUNCH:-auto_drone gazebo_px4_autonomy.launch.py}"
GZ_PREFIX="${GZ_PREFIX:-}"
GZ_IP="${GZ_IP:-127.0.0.1}"
GZ_PARTITION="${GZ_PARTITION:-auto_drone_px4}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-default}"
PX4_GZ_STANDALONE="${PX4_GZ_STANDALONE:-0}"
START_GZ_SERVER="${START_GZ_SERVER:-0}"
DRY_RUN=0
SKIP_PX4=0
SKIP_AGENT=0

usage() {
  cat <<USAGE
Usage: $0 [--dry-run] [--skip-px4] [--skip-agent]

Environment:
  PX4_DIR=$PX4_DIR
  ROS_SETUP=$ROS_SETUP
  WORKSPACE_SETUP=$WORKSPACE_SETUP
  PX4_TARGET=$PX4_TARGET
  XRCE_PORT=$XRCE_PORT
  AUTONOMY_LAUNCH=$AUTONOMY_LAUNCH
  GZ_PREFIX=$GZ_PREFIX
  GZ_IP=$GZ_IP
  GZ_PARTITION=$GZ_PARTITION
  PX4_GZ_WORLD=$PX4_GZ_WORLD
  PX4_GZ_STANDALONE=$PX4_GZ_STANDALONE
  START_GZ_SERVER=$START_GZ_SERVER
USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --dry-run) DRY_RUN=1 ;;
    --skip-px4) SKIP_PX4=1 ;;
    --skip-agent) SKIP_AGENT=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

require_file() {
  if [[ ! -f "$1" ]]; then
    if [[ "$DRY_RUN" -eq 1 ]]; then
      echo "Dry-run warning: missing file: $1" >&2
      return 0
    fi
    echo "Missing required file: $1" >&2
    exit 1
  fi
}

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    if [[ "$DRY_RUN" -eq 1 ]]; then
      echo "Dry-run warning: missing command: $1" >&2
      return 0
    fi
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

run_or_print() {
  echo "+ $*"
  if [[ "$DRY_RUN" -eq 0 ]]; then
    "$@"
  fi
}

require_file "$ROS_SETUP"
require_file "$WORKSPACE_SETUP"
require_cmd bash

if [[ -f "$ROS_SETUP" ]]; then
  # shellcheck source=/dev/null
  HAD_NOUNSET=0
  case "$-" in
    *u*) HAD_NOUNSET=1; set +u ;;
  esac
  source "$ROS_SETUP"
  if [[ "$HAD_NOUNSET" -eq 1 ]]; then
    set -u
  fi
fi

require_cmd ros2

configure_gazebo_env() {
  export GZ_IP
  export GZ_PARTITION
  export PX4_GZ_WORLD

  if [[ -n "$GZ_PREFIX" ]]; then
    if [[ -d "$GZ_PREFIX/share/gz" && -w "$GZ_PREFIX/share/gz" ]]; then
      while IFS= read -r -d '' yaml_file; do
        sed -i "s#library_path: /usr/lib/ruby/gz/#library_path: $GZ_PREFIX/lib/ruby/gz/#" "$yaml_file"
      done < <(find "$GZ_PREFIX/share/gz" -maxdepth 1 -name "*.yaml" -type f -print0)
    fi
    export PATH="$GZ_PREFIX/bin:$HOME/.local/bin:$PATH"
    export CMAKE_PREFIX_PATH="$GZ_PREFIX:${CMAKE_PREFIX_PATH:-}"
    export PKG_CONFIG_PATH="$GZ_PREFIX/lib/x86_64-linux-gnu/pkgconfig:$GZ_PREFIX/share/pkgconfig:${PKG_CONFIG_PATH:-}"
    export LD_LIBRARY_PATH="$GZ_PREFIX/lib/x86_64-linux-gnu:$GZ_PREFIX/lib/x86_64-linux-gnu/gz-sim-8/plugins:$GZ_PREFIX/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins:${LD_LIBRARY_PATH:-}"
    export GZ_CONFIG_PATH="$GZ_PREFIX/share/gz:${GZ_CONFIG_PATH:-}"
    export RUBYLIB="$GZ_PREFIX/lib/ruby:$GZ_PREFIX/lib/x86_64-linux-gnu/ruby:${RUBYLIB:-}"
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$GZ_PREFIX/lib/x86_64-linux-gnu/gz-sim-8/plugins:$GZ_PREFIX/lib/x86_64-linux-gnu:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
    export GZ_SIM_PHYSICS_ENGINE_PATH="$GZ_PREFIX/lib/x86_64-linux-gnu/gz-physics-7/engine-plugins:$GZ_PREFIX/lib/x86_64-linux-gnu/gz-physics-6/engine-plugins:${GZ_SIM_PHYSICS_ENGINE_PATH:-}"
  fi

  if [[ -d "$PX4_DIR/Tools/simulation/gz/models" ]]; then
    export GZ_SIM_RESOURCE_PATH="$PX4_DIR/Tools/simulation/gz/models:$PX4_DIR/Tools/simulation/gz/worlds:${GZ_SIM_RESOURCE_PATH:-}"
  fi
}

if [[ "$SKIP_PX4" -eq 0 ]]; then
  if [[ ! -d "$PX4_DIR" ]]; then
    if [[ "$DRY_RUN" -eq 1 ]]; then
      echo "Dry-run warning: missing PX4 checkout: $PX4_DIR" >&2
    else
    echo "Missing PX4 checkout: $PX4_DIR" >&2
    exit 1
    fi
  fi
  if [[ -d "$PX4_DIR" && ! -f "$PX4_DIR/Makefile" ]]; then
    if [[ "$DRY_RUN" -eq 1 ]]; then
      echo "Dry-run warning: PX4_DIR does not look like PX4-Autopilot: $PX4_DIR" >&2
    else
    echo "PX4_DIR does not look like PX4-Autopilot: $PX4_DIR" >&2
    exit 1
    fi
  fi
fi

configure_gazebo_env

if [[ "$SKIP_AGENT" -eq 0 ]]; then
  require_cmd MicroXRCEAgent
fi

if [[ "$SKIP_PX4" -eq 0 && ("$START_GZ_SERVER" -eq 1 || "$PX4_GZ_STANDALONE" == "1") ]]; then
  require_cmd gz
fi

PIDS=()
cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" >/dev/null 2>&1; then
      kill "$pid" >/dev/null 2>&1 || true
    fi
  done
}
trap cleanup EXIT

if [[ "$SKIP_AGENT" -eq 0 ]]; then
  echo "+ MicroXRCEAgent udp4 -p $XRCE_PORT"
  if [[ "$DRY_RUN" -eq 0 ]]; then
    MicroXRCEAgent udp4 -p "$XRCE_PORT" &
    PIDS+=("$!")
    sleep 2
  fi
fi

if [[ "$SKIP_PX4" -eq 0 ]]; then
  if [[ "$START_GZ_SERVER" -eq 1 || "$PX4_GZ_STANDALONE" == "1" ]]; then
    export PX4_GZ_STANDALONE=1
    echo "+ gz sim -r -s $PX4_DIR/Tools/simulation/gz/worlds/$PX4_GZ_WORLD.sdf"
    if [[ "$DRY_RUN" -eq 0 ]]; then
      gz sim -r -s "$PX4_DIR/Tools/simulation/gz/worlds/$PX4_GZ_WORLD.sdf" &
      PIDS+=("$!")
      sleep 8
    fi
  fi

  echo "+ cd $PX4_DIR && make px4_sitl $PX4_TARGET"
  if [[ "$DRY_RUN" -eq 0 ]]; then
    (cd "$PX4_DIR" && make px4_sitl "$PX4_TARGET") &
    PIDS+=("$!")
    sleep 12
  fi
fi

run_or_print bash -lc "source '$ROS_SETUP' && source '$WORKSPACE_SETUP' && ros2 launch $AUTONOMY_LAUNCH"
