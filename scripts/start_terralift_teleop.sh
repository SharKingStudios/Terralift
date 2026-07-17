#!/usr/bin/env bash
set -eo pipefail

LOG_DIR="$HOME/terralift_ws/log"
TELEOP_CONFIG="$HOME/teleop_mecanum.yaml"
STAMP="$(date +%Y%m%d_%H%M%S)"
ROBOT_SSH="${TERRALIFT_ROBOT_SSH:-ubuntu@rospi.local}"
ROBOT_SSH_FALLBACKS="${TERRALIFT_ROBOT_SSH_FALLBACKS:-ubuntu@10.42.0.1 ubuntu@10.0.0.38}"
ACTIVE_ROBOT_SSH="$ROBOT_SSH"
ROBOT_NAV2_PARAMS="${TERRALIFT_NAV2_PARAMS:-dwb_smac2d_course_valid.yaml}"
ROBOT_USE_APRILTAGS="${TERRALIFT_USE_APRILTAGS:-false}"
ROBOT_WIFI_PROFILE="${TERRALIFT_WIFI_PROFILE:-Terralift}"
HOME_WIFI_PROFILE="${TERRALIFT_HOME_WIFI_PROFILE:-PP}"
PRIMARY_WIFI_DEVICE="${TERRALIFT_PRIMARY_WIFI:-wlp1s0}"
TRY_DONGLE_HOME="${TERRALIFT_TRY_DONGLE_HOME:-true}"
AUTO_WIFI="${TERRALIFT_AUTO_WIFI:-false}"
WIFI_ONLY="false"
JUMPSTART_ONLY="false"
WIFI_ONLY_WAIT_SEC="${TERRALIFT_WIFI_ONLY_WAIT_SEC:-0}"
DDS_CONFIG="/tmp/terralift_cyclonedds_${USER}.xml"
STATUS_ONLY="false"
NO_ROBOT="false"
DDS_CONFIGURED="false"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --status)
      STATUS_ONLY="true"
      shift
      ;;
    --no-robot)
      NO_ROBOT="true"
      shift
      ;;
    --wifi-only)
      WIFI_ONLY="true"
      NO_ROBOT="true"
      AUTO_WIFI="true"
      shift
      ;;
    --jumpstart-only)
      JUMPSTART_ONLY="true"
      AUTO_WIFI="false"
      shift
      ;;
    --wifi-wait)
      WIFI_ONLY_WAIT_SEC="${2:-15}"
      shift 2
      ;;
    -h|--help)
      echo "Usage: $0 [--status] [--no-robot] [--wifi-only] [--jumpstart-only] [--wifi-wait SECONDS]"
      echo "Run with no arguments for the arrow-key launcher menu."
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

mkdir -p "$LOG_DIR"

source_ros() {
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
  else
    echo "Missing /opt/ros/jazzy/setup.bash"
    return 1
  fi

  if [[ -f "$HOME/terralift_ws/install/setup.bash" ]]; then
    source "$HOME/terralift_ws/install/setup.bash"
  fi
}

wifi_devices() {
  nmcli -t -f DEVICE,TYPE device 2>/dev/null | awk -F: '$2 == "wifi" { print $1 }'
}

active_robot_iface() {
  ip -o -4 addr show scope global | awk '$4 ~ /^10[.]42[.]0[.]/ { split($4, a, "/"); print $2 " " a[1]; exit }'
}

iface_has_robot_addr() {
  local iface="$1"
  ip -o -4 addr show dev "$iface" scope global 2>/dev/null | awk '$4 ~ /^10[.]42[.]0[.]/ { found=1 } END { exit(found ? 0 : 1) }'
}

iface_connection() {
  local iface="$1"
  nmcli -t -f DEVICE,CONNECTION device 2>/dev/null | awk -F: -v iface="$iface" '$1 == iface { print $2; exit }'
}

select_primary_wifi() {
  if nmcli -t -f DEVICE,TYPE device 2>/dev/null | grep -qxF "$PRIMARY_WIFI_DEVICE:wifi"; then
    echo "$PRIMARY_WIFI_DEVICE"
    return 0
  fi

  wifi_devices | awk '$1 !~ /^wlx/ { print; exit }'
}

select_aux_wifi() {
  local primary="$1"
  wifi_devices | awk -v primary="$primary" '$1 != primary { print; exit }'
}

ensure_robot_wifi() {
  if ! command -v nmcli >/dev/null 2>&1; then
    echo "nmcli is unavailable; cannot auto-connect to $ROBOT_WIFI_PROFILE."
    return 1
  fi

  if ! nmcli -t -f NAME connection show | grep -qxF "$ROBOT_WIFI_PROFILE"; then
    echo "NetworkManager profile '$ROBOT_WIFI_PROFILE' is not saved."
    return 1
  fi

  local primary current_robot
  primary="$(select_primary_wifi || true)"
  if [[ -z "$primary" ]]; then
    echo "No Wi-Fi device found for robot network."
    return 1
  fi

  if iface_has_robot_addr "$primary"; then
    echo "Robot link: main Wi-Fi '$primary' is already on '$ROBOT_WIFI_PROFILE'."
    return 0
  fi

  current_robot="$(active_robot_iface | awk '{print $1}' || true)"
  if [[ -n "$current_robot" && "$current_robot" != "$primary" ]]; then
    echo "Robot link is currently on '$current_robot'; moving robot network to main Wi-Fi '$primary'."
  else
    echo "Robot link: connecting main Wi-Fi '$primary' to '$ROBOT_WIFI_PROFILE'."
  fi

  nmcli connection up "$ROBOT_WIFI_PROFILE" ifname "$primary" || true

  for _ in {1..24}; do
    if iface_has_robot_addr "$primary"; then
      return 0
    fi
    sleep 0.5
  done

  return 1
}

try_home_on_aux_wifi() {
  [[ "$TRY_DONGLE_HOME" == "true" ]] || return 0
  command -v nmcli >/dev/null 2>&1 || return 0
  nmcli -t -f NAME connection show | grep -qxF "$HOME_WIFI_PROFILE" || return 0

  local robot_iface primary aux clone
  robot_iface="$(active_robot_iface | awk '{print $1}' || true)"
  primary="$(select_primary_wifi || true)"
  aux="$(select_aux_wifi "$primary" || true)"

  [[ -n "$robot_iface" && "$robot_iface" == "$primary" ]] || return 0
  [[ -n "$aux" ]] || return 0

  local aux_conn
  aux_conn="$(iface_connection "$aux" || true)"
  if [[ "$aux_conn" == "$HOME_WIFI_PROFILE" || "$aux_conn" == "$HOME_WIFI_PROFILE-$aux" ]]; then
    echo "Secondary Wi-Fi: '$aux' is already on home profile '$aux_conn'."
    return 0
  fi

  clone="${HOME_WIFI_PROFILE}-${aux}"
  echo "Secondary Wi-Fi: trying to keep '$HOME_WIFI_PROFILE' available on '$aux' as '$clone' (best effort)."

  if ! nmcli -t -f NAME connection show | grep -qxF "$clone"; then
    nmcli connection clone "$HOME_WIFI_PROFILE" "$clone" >/dev/null 2>&1 || return 0
    nmcli connection modify "$clone" connection.interface-name "$aux" connection.autoconnect no >/dev/null 2>&1 || true
  fi

  nmcli connection up "$clone" ifname "$aux" >/dev/null 2>&1 || true
}

ssh_host() {
  local candidate="$1"
  candidate="${candidate#*@}"
  echo "$candidate"
}

is_ipv4_literal() {
  [[ "$1" =~ ^[0-9]+[.][0-9]+[.][0-9]+[.][0-9]+$ ]]
}

resolve_ipv4() {
  local host="$1"
  if is_ipv4_literal "$host"; then
    echo "$host"
    return 0
  fi

  if command -v getent >/dev/null 2>&1; then
    getent ahostsv4 "$host" | awk '{ print $1; exit }'
    return 0
  fi

  echo "$host"
}

iface_addr_for_route() {
  local host="$1"
  local ip route iface addr
  ip="$(resolve_ipv4 "$host")"
  [[ -n "$ip" ]] || return 1

  route="$(ip -o route get "$ip" 2>/dev/null | head -n 1)" || return 1
  iface="$(awk '{ for (i = 1; i <= NF; i++) if ($i == "dev") { print $(i + 1); exit } }' <<<"$route")"
  addr="$(awk '{ for (i = 1; i <= NF; i++) if ($i == "src") { print $(i + 1); exit } }' <<<"$route")"

  if [[ -n "$iface" && -z "$addr" ]]; then
    addr="$(ip -o -4 addr show dev "$iface" scope global 2>/dev/null | awk '{ split($4, a, "/"); print a[1]; exit }')"
  fi

  [[ -n "$iface" && -n "$addr" ]] || return 1
  echo "$iface $addr $host"
}

routed_robot_iface() {
  local candidate host routed
  while IFS= read -r candidate; do
    [[ -n "$candidate" ]] || continue
    host="$(ssh_host "$candidate")"
    routed="$(iface_addr_for_route "$host" || true)"
    if [[ -n "$routed" ]]; then
      echo "$routed"
      return 0
    fi
  done < <(robot_ssh_candidates)
}
configure_dds() {
  local iface_addr iface addr routed rest source_desc
  routed="$(iface_addr_for_route "$(ssh_host "$ACTIVE_ROBOT_SSH")" || true)"
  if [[ -z "$routed" ]]; then
    routed="$(routed_robot_iface || true)"
  fi

  if [[ -n "$routed" ]]; then
    iface="${routed%% *}"
    rest="${routed#* }"
    addr="${rest%% *}"
    source_desc="route to ${rest#* }"
  else
    iface_addr="$(active_robot_iface || true)"
    if [[ -n "$iface_addr" ]]; then
      iface="${iface_addr%% *}"
      addr="${iface_addr##* }"
      source_desc="Terralift AP interface"
    fi
  fi

  if [[ -n "${iface:-}" && -n "${addr:-}" ]]; then
    cat > "$DDS_CONFIG" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface name="$iface" address="$addr" />
      </Interfaces>
      <DontRoute>true</DontRoute>
      <AllowMulticast>spdp</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>
EOF
    export CYCLONEDDS_URI="file://$DDS_CONFIG"
    echo "CycloneDDS pinned to $iface at $addr ($source_desc)"
    DDS_CONFIGURED="true"
  elif [[ -f "$HOME/cyclonedds.xml" ]]; then
    export CYCLONEDDS_URI="file://$HOME/cyclonedds.xml"
    echo "WARNING: could not auto-select a DDS interface; falling back to $CYCLONEDDS_URI"
    DDS_CONFIGURED="true"
  else
    unset CYCLONEDDS_URI || true
    echo "WARNING: could not auto-select a DDS interface and no $HOME/cyclonedds.xml exists. DDS will use defaults."
    DDS_CONFIGURED="true"
  fi

  if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
    ros2 daemon start >/dev/null 2>&1 || true
  fi
}

robot_ssh_candidates() {
  local candidate seen=""

  emit_candidate() {
    local item="$1"
    [[ -n "$item" ]] || return 0
    case " $seen " in
      *" $item "*) return 0;;
    esac
    seen="$seen $item"
    printf '%s\n' "$item"
  }

  if active_robot_iface >/dev/null; then
    emit_candidate "ubuntu@10.42.0.1"
  fi

  emit_candidate "$ROBOT_SSH"
  for candidate in $ROBOT_SSH_FALLBACKS; do
    emit_candidate "$candidate"
  done
}
jumpstart_robot() {
  if [[ "$NO_ROBOT" == "true" ]]; then
    echo "Skipping robot jumpstart (--no-robot)."
    return 0
  fi

  local candidate
  while IFS= read -r candidate; do
    [[ -n "$candidate" ]] || continue
    echo "Jumpstarting robot demo on $candidate"
    if ssh -o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new "$candidate" \
        "~/terralift_ws/bin/start_demo_mode.sh --nav2-params '$ROBOT_NAV2_PARAMS' --use-apriltags '$ROBOT_USE_APRILTAGS'"; then
      ACTIVE_ROBOT_SSH="$candidate"
      if [[ "$DDS_CONFIGURED" == "true" ]]; then
        configure_dds
      fi
      echo "Robot demo start command completed via $ACTIVE_ROBOT_SSH."
      return 0
    fi
  done < <(robot_ssh_candidates)

  echo "WARNING: could not jumpstart robot demo over SSH. Continuing with laptop teleop/RViz."
}

open_robot_output_terminal() {
  if [[ "$NO_ROBOT" == "true" ]]; then
    return 0
  fi

  local remote_cmd ssh_line terminal_line
  remote_cmd="trap 'echo; echo Stopping Terralift stack on robot...; ~/terralift_ws/bin/stop_demo_mode.sh; exit 130' INT TERM; tail -n 160 -F ~/terralift_run_logs/terralift_demo.log & wait \$!"
  printf -v ssh_line 'ssh -t -o StrictHostKeyChecking=accept-new %q bash -lc %q' "$ACTIVE_ROBOT_SSH" "$remote_cmd"
  terminal_line="$ssh_line; echo; read -r -p 'Robot output ended. Press Enter to close...'"

  if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    echo "No GUI display detected for opening a robot-output terminal."
    echo "Manual robot output/stop: $ssh_line"
    return 0
  fi

  echo "Opening robot output terminal following $ACTIVE_ROBOT_SSH:terralift_demo log"
  if command -v konsole >/dev/null 2>&1; then
    konsole --new-tab -p tabtitle='Terralift Robot' -e bash -lc "$terminal_line" >/dev/null 2>&1 &
  elif command -v xterm >/dev/null 2>&1; then
    xterm -T 'Terralift Robot' -e bash -lc "$terminal_line" >/dev/null 2>&1 &
  else
    echo "No konsole/xterm found. Manual robot output/stop: $ssh_line"
  fi
}

show_status() {
  echo "=== laptop network ==="
  ip -brief addr
  echo
  echo "=== NetworkManager Wi-Fi devices ==="
  nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device 2>/dev/null | grep ':wifi:' || true
  echo
  echo "=== robot AP interface for DDS ==="
  active_robot_iface || true
  echo
  echo "=== robot status ==="
  ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new "$ROBOT_SSH" \
    "~/terralift_ws/bin/start_demo_mode.sh --status" || true
  echo
  echo "=== local ROS packages ==="
  source_ros || true
  ros2 pkg list 2>/dev/null | grep -E '^(joy|teleop_twist_joy|rviz2|terralift)$' || true
}

warn_robot_network() {
  local routed rest
  if active_robot_iface >/dev/null; then
    return 0
  fi

  echo "WARNING: this laptop does not currently have a 10.42.0.x Terralift robot-network address."
  routed="$(routed_robot_iface || true)"
  if [[ -n "$routed" ]]; then
    rest="${routed#* }"
    echo "It can route to ${rest#* } via ${routed%% *} at ${rest%% *}, so home-network ROS may still work."
  fi
  echo "ROS discovery may not see /scan, /map, /tf, or /odom from the robot."
  echo "You can still continue if you are intentionally reaching the robot another way."
  echo
}

choose_menu() {
  local title="$1"
  shift
  local options=("$@")
  local selected=0
  local key rest i

  while true; do
    printf '\033[2J\033[H'
    echo "$title"
    echo "Use Up/Down, Enter to select, q to quit."
    echo
    for i in "${!options[@]}"; do
      if [[ "$i" -eq "$selected" ]]; then
        printf '  > %s\n' "${options[$i]}"
      else
        printf '    %s\n' "${options[$i]}"
      fi
    done

    IFS= read -rsn1 key || return 1
    case "$key" in
      '')
        MENU_CHOICE="$selected"
        return 0
        ;;
      q|Q)
        echo
        exit 0
        ;;
      $'\x1b')
        IFS= read -rsn2 -t 0.1 rest || rest=""
        case "$rest" in
          '[A') selected=$((selected - 1));;
          '[B') selected=$((selected + 1));;
        esac
        ;;
    esac

    if (( selected < 0 )); then
      selected=$((${#options[@]} - 1))
    elif (( selected >= ${#options[@]} )); then
      selected=0
    fi
  done
}

run_launcher_menu() {
  local actions=(
    "Full teleop: jumpstart robot, show robot output, start Xbox teleop + RViz"
    "Jumpstart robot only"
    "Laptop teleop + RViz only (do not SSH-start robot)"
    "Status only"
    "Exit"
  )
  local nav2_options=(
    "dwb_smac2d_course_valid.yaml"
    "rpp_smachybrid.yaml"
    "dwb_smac2d.yaml"
    "dwb_smachybrid.yaml"
  )

  warn_robot_network
  read -r -p "Press Enter to open the Terralift launcher menu..."

  choose_menu "Terralift Launcher" "${actions[@]}"
  case "$MENU_CHOICE" in
    0)
      NO_ROBOT="false"
      JUMPSTART_ONLY="false"
      ;;
    1)
      JUMPSTART_ONLY="true"
      AUTO_WIFI="false"
      ;;
    2)
      NO_ROBOT="true"
      ;;
    3)
      STATUS_ONLY="true"
      ;;
    4)
      exit 0
      ;;
  esac

  if [[ "$NO_ROBOT" != "true" || "$JUMPSTART_ONLY" == "true" ]]; then
    choose_menu "Select Nav2 Config" "${nav2_options[@]}"
    ROBOT_NAV2_PARAMS="${nav2_options[$MENU_CHOICE]}"
  fi

  printf '\033[2J\033[H'
}

if [[ "$STATUS_ONLY" != "true" && "$WIFI_ONLY" != "true" && "$JUMPSTART_ONLY" != "true" && "$NO_ROBOT" != "true" && "$#" -eq 0 ]]; then
  run_launcher_menu
fi

if [[ "$STATUS_ONLY" == "true" ]]; then
  show_status
  exit 0
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-17}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

source_ros

first_robot_candidate=""
while IFS= read -r first_robot_candidate; do
  break
done < <(robot_ssh_candidates)
if [[ -n "$first_robot_candidate" ]]; then
  ACTIVE_ROBOT_SSH="$first_robot_candidate"
fi

echo "Terralift teleop launcher"
echo "Robot Wi-Fi profile: $ROBOT_WIFI_PROFILE on main Wi-Fi $PRIMARY_WIFI_DEVICE (auto Wi-Fi: $AUTO_WIFI; Wi-Fi Fix is separate)"
echo "Robot SSH: $ROBOT_SSH"
echo "Robot launch: terralift_demo.launch.py use_apriltags:=$ROBOT_USE_APRILTAGS nav2_params:=$ROBOT_NAV2_PARAMS"
echo "Teleop config: $TELEOP_CONFIG"
echo

if [[ "$JUMPSTART_ONLY" == "true" ]]; then
  echo "Jumpstart-only mode: not changing Wi-Fi. Use Terralift Wi-Fi Fix if adapter state needs repair."
elif [[ "$AUTO_WIFI" == "true" ]]; then
  ensure_robot_wifi || echo "WARNING: robot AP Wi-Fi is not active on the main Wi-Fi. ROS discovery may not reach the robot."
  try_home_on_aux_wifi
else
  echo "Auto Wi-Fi switching is disabled; assuming you already connected this laptop to the Terralift network."
  active_robot_iface >/dev/null || echo "WARNING: no active 10.42.0.x interface found. ROS discovery may not reach the robot."
fi
configure_dds

echo "ROS_DISTRO=${ROS_DISTRO:-unknown} ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION CYCLONEDDS_URI=${CYCLONEDDS_URI:-default}"
echo

if [[ "$WIFI_ONLY" == "true" ]]; then
  echo "Wi-Fi-only mode complete. Current Wi-Fi state:"
  nmcli -t -f DEVICE,TYPE,STATE,CONNECTION device 2>/dev/null | grep ':wifi:' || true
  echo
  if [[ "$WIFI_ONLY_WAIT_SEC" =~ ^[0-9]+$ && "$WIFI_ONLY_WAIT_SEC" -gt 0 ]]; then
    echo "Waiting ${WIFI_ONLY_WAIT_SEC}s so you can see the result, then this window will close."
    sleep "$WIFI_ONLY_WAIT_SEC"
  fi
  exit 0
fi

if [[ "$JUMPSTART_ONLY" == "true" ]]; then
  jumpstart_robot
  open_robot_output_terminal
  echo
  echo "Robot jumpstart command finished. The robot output terminal should be open if a GUI terminal is available."
  if [[ -t 0 ]]; then
    read -r -p "Press Enter to close this launcher window..."
  fi
  exit 0
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 is not on PATH after sourcing setup files."
  read -r -p "Press Enter to close..."
  exit 1
fi

if [[ ! -f "$TELEOP_CONFIG" ]]; then
  echo "Missing teleop config: $TELEOP_CONFIG"
  read -r -p "Press Enter to close..."
  exit 1
fi

for pkg in joy teleop_twist_joy rviz2; do
  if ! ros2 pkg prefix "$pkg" >/dev/null 2>&1; then
    echo "Missing ROS package: $pkg"
    read -r -p "Press Enter to close..."
    exit 1
  fi
done

jumpstart_robot
open_robot_output_terminal

if ! ls /dev/input/js* >/dev/null 2>&1; then
  echo "WARNING: no /dev/input/js* controller device is visible yet."
  echo "Pair/plug the Xbox controller, then restart this launcher if joy_node cannot open it."
  echo
fi

cleanup() {
  echo
  echo "Stopping laptop teleop processes..."
  [[ -n "${JOY_PID:-}" ]] && kill "$JOY_PID" 2>/dev/null || true
  [[ -n "${TELEOP_PID:-}" ]] && kill "$TELEOP_PID" 2>/dev/null || true
  [[ -n "${RVIZ_PID:-}" ]] && kill "$RVIZ_PID" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Starting joy_node -> /joy"
ros2 run joy joy_node --ros-args \
  -p autorepeat_rate:=20.0 \
  -p coalesce_interval_ms:=1 \
  > >(tee "$LOG_DIR/joy_node_$STAMP.log") 2>&1 &
JOY_PID=$!

sleep 1

echo "Starting teleop_twist_joy -> /cmd_vel_teleop"
ros2 run teleop_twist_joy teleop_node --ros-args \
  --params-file "$TELEOP_CONFIG" \
  -r cmd_vel:=/cmd_vel_teleop \
  > >(tee "$LOG_DIR/teleop_twist_joy_$STAMP.log") 2>&1 &
TELEOP_PID=$!

sleep 1

echo "Starting RViz2"
rviz2 > >(tee "$LOG_DIR/rviz2_$STAMP.log") 2>&1 &
RVIZ_PID=$!

echo
echo "Running. Your teleop_mecanum.yaml controls the Xbox mapping."
echo "Robot demo stack is in tmux on the robot. Output window follows ~/terralift_run_logs/terralift_demo.log."
echo "Close this window or press Ctrl+C to stop laptop teleop/RViz. Robot demo keeps running until stopped."
echo

wait "$RVIZ_PID"
