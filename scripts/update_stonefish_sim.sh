#!/bin/bash
# Set up or update the Stonefish simulator stack: Stonefish 1.6 with the scene editor, the
# simulator's repos on main, and a build of the simulator packages. The mission stack (vortex-auv,
# vortex-cv) is not touched.
#
# Safe to run again: repos that are up to date are left alone, and Stonefish is only rebuilt
# when its source changed. First time, without the repo:
#   bash <(curl -fsSL https://raw.githubusercontent.com/vortexntnu/vortex-stonefish-sim/main/scripts/update_stonefish_sim.sh)

set -euo pipefail

# This script is in a repo it updates: run a copy, so git can change the file under it
if [[ -z "${UPDATE_SIM_SCRIPT_DIR:-}" && -f "${BASH_SOURCE[0]}" ]]; then
    copy="$(mktemp)"
    cp "${BASH_SOURCE[0]}" "$copy"
    UPDATE_SIM_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" exec bash "$copy" "$@"
fi

usage() {
    cat <<EOF
Usage: $(basename "$0") [OPTIONS]

  --ws <dir>          ROS 2 workspace (default: found from this script, then ~/ros2_ws).
  --stonefish <dir>   Stonefish source (default: found in the usual places, else cloned to ~/stonefish).
  --prefix <dir>      Install Stonefish here instead of /usr/local (no sudo needed),
                      e.g. ~/.local/stonefish.
  --jobs <n>          Parallel build jobs (default: from your RAM, about 4 GB each).
  --stash             Stash uncommitted changes instead of stopping.
  --skip-stonefish    Do not build Stonefish.
  --skip-build        Only update the repos.
  --dry-run           Show what would be done.
  -h, --help          Show this help message.
EOF
}

# Repo in src/ and the branch it must be on
REPOS=(
    "stonefish_ros2:main"
    "vortex-stonefish-sim:main"
    "vortex-msgs:main"
    "vortex-utils:main"
)
STONEFISH_BRANCH="main"
STONEFISH_VERSION="1.6.0"
GITHUB="vortexntnu"

WS=""
SF=""
PREFIX="/usr/local"
JOBS=""
STASH=false
SKIP_SF=false
SKIP_BUILD=false
DRY=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --ws)             WS="$2"; shift 2 ;;
        --stonefish)      SF="$2"; shift 2 ;;
        --prefix)         PREFIX="$2"; shift 2 ;;
        --jobs)           JOBS="$2"; shift 2 ;;
        --stash)          STASH=true; shift ;;
        --skip-stonefish) SKIP_SF=true; shift ;;
        --skip-build)     SKIP_BUILD=true; shift ;;
        --dry-run)        DRY=true; shift ;;
        -h|--help)        usage; exit 0 ;;
        *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
done

step() { echo -e "\n\033[1;34m==> $*\033[0m"; }
note() { echo -e "    $*"; }
warn() { echo -e "\033[1;33m    $*\033[0m"; }
die()  { echo -e "\033[1;31mError: $*\033[0m" >&2; exit 1; }
run()  { if $DRY; then echo "    [dry-run] $*"; else "$@"; fi; }

# --- Find things ---------------------------------------------------------------------

step "Finding the workspace and Stonefish"

is_ws() { [[ -d "$1/src" ]]; }
if [[ -z "$WS" ]]; then
    here="${UPDATE_SIM_SCRIPT_DIR:-}"
    # <ws>/src/vortex-stonefish-sim/scripts
    if [[ "$here" == */src/*/scripts ]] && is_ws "${here%/src/*}"; then
        WS="${here%/src/*}"
    else
        for c in "${ROS_WS:-}" ~/ros2_ws ~/vortex_ws ~/ws; do
            [[ -n "$c" ]] && is_ws "$c" && { WS="$c"; break; }
        done
    fi
fi
[[ -n "$WS" ]] && is_ws "$WS" || die "no ROS 2 workspace found, pass --ws <dir> (the folder with src/)"
WS="$(cd "$WS" && pwd)"
note "workspace: $WS"

is_sf() { [[ -f "$1/CMakeLists.txt" ]] && grep -q "project(Stonefish" "$1/CMakeLists.txt"; }
if [[ -z "$SF" ]]; then
    for c in "$WS/src/stonefish" "$WS/src/Stonefish" ~/stonefish ~/Stonefish "$WS/../stonefish" ~/src/stonefish ~/git/stonefish ~/repos/stonefish; do
        is_sf "$c" && { SF="$c"; break; }
    done
fi
if [[ -z "$SF" ]]; then
    found="$(find ~ -maxdepth 6 -name CMakeLists.txt -path '*tonefish*' -not -path '*/build/*' 2>/dev/null |
             while read -r f; do d="$(dirname "$f")"; is_sf "$d" && echo "$d" && break; done || true)"
    SF="$found"
fi
if [[ -n "$SF" ]]; then
    SF="$(cd "$SF" && pwd)"
    note "stonefish: $SF"
else
    SF="$HOME/stonefish"
    note "stonefish: not found, will clone to $SF"
fi

# SSH or HTTPS, like the workspace's other Vortex repos
URL_BASE="https://github.com/$GITHUB"
for r in "$WS"/src/vortex-*; do
    if git -C "$r" remote get-url origin 2>/dev/null | grep -q '^git@github.com'; then
        URL_BASE="git@github.com:$GITHUB"
        break
    fi
done

[[ -z "$JOBS" ]] && JOBS=$(( $(free -g | awk '/^Mem:/ {print $2}') / 4 ))
(( JOBS < 1 )) && JOBS=1
(( JOBS > $(nproc) )) && JOBS=$(nproc)
note "build jobs: $JOBS"

# --- Check before changing anything -----------------------------------------------------

step "Checking for uncommitted changes"
dirty=()
for entry in "${REPOS[@]}"; do
    d="$WS/src/${entry%%:*}"
    [[ -d "$d/.git" ]] && [[ -n "$(git -C "$d" status --porcelain)" ]] && dirty+=("$d")
done
[[ -d "$SF/.git" ]] && [[ -n "$(git -C "$SF" status --porcelain)" ]] && dirty+=("$SF")
if (( ${#dirty[@]} )); then
    if $STASH; then
        for d in "${dirty[@]}"; do
            note "stashing changes in $d"
            run git -C "$d" stash push -u -m "update_stonefish_sim $(date +%F_%H%M)"
        done
    else
        warn "These repos have uncommitted changes:"
        for d in "${dirty[@]}"; do note "  $d"; done
        die "commit them, or run again with --stash to stash them"
    fi
else
    note "none"
fi

if ! $SKIP_SF; then
    missing=()
    for p in libglm-dev libsdl2-dev libfreetype6-dev cmake g++; do
        dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p")
    done
    if (( ${#missing[@]} )); then
        step "Installing Stonefish's build dependencies: ${missing[*]}"
        run sudo apt-get install -y "${missing[@]}"
    fi
fi

# --- Repos -------------------------------------------------------------------------------

# Clone, or check out the branch and move it to the remote one. When the remote branch was
# rewritten (it can not fast-forward), the old local branch is kept as backup/<branch>-<date>.
update_repo() {
    local dir="$1" name="$2" branch="$3" remote="origin"
    if [[ ! -d "$dir/.git" ]]; then
        note "cloning $name ($branch)"
        run git clone -b "$branch" "$URL_BASE/$name.git" "$dir"
        return
    fi
    # Stonefish clones from the original author: use the Vortex fork as another remote
    if [[ "$name" == stonefish ]] && ! git -C "$dir" remote get-url origin | grep -qi "$GITHUB/"; then
        remote="vortex"
        git -C "$dir" remote get-url vortex >/dev/null 2>&1 || run git -C "$dir" remote add vortex "$URL_BASE/stonefish.git"
    fi
    run git -C "$dir" fetch -q "$remote"
    if ! git -C "$dir" rev-parse -q --verify "$remote/$branch" >/dev/null; then
        $DRY && { note "$name: would check out $branch"; return; }
        die "$name has no branch $remote/$branch"
    fi
    if git -C "$dir" rev-parse -q --verify "refs/heads/$branch" >/dev/null; then
        [[ "$(git -C "$dir" branch --show-current)" != "$branch" ]] && run git -C "$dir" checkout -q "$branch"
    else
        run git -C "$dir" checkout -q -b "$branch" --track "$remote/$branch"
        $DRY && return
    fi
    local local_ref remote_ref
    local_ref="$(git -C "$dir" rev-parse "refs/heads/$branch")"
    remote_ref="$(git -C "$dir" rev-parse "$remote/$branch")"
    if [[ "$local_ref" == "$remote_ref" ]]; then
        note "$name: up to date ($branch)"
    elif git -C "$dir" merge-base --is-ancestor "$local_ref" "$remote_ref"; then
        note "$name: updating $branch"
        run git -C "$dir" merge -q --ff-only "$remote/$branch"
    elif git -C "$dir" merge-base --is-ancestor "$remote_ref" "$local_ref"; then
        warn "$name: you have commits on $branch that are not pushed, leaving it as it is"
    else
        local backup
        backup="backup/$branch-$(date +%F_%H%M)"
        warn "$name: $branch was rewritten on GitHub; your old one is kept as $backup"
        run git -C "$dir" branch -q "$backup" "$local_ref"
        run git -C "$dir" reset -q --hard "$remote/$branch"
    fi
}

step "Updating the repos"
SF_BEFORE="$(git -C "$SF" rev-parse HEAD 2>/dev/null || echo none)"
update_repo "$SF" stonefish "$STONEFISH_BRANCH"
for entry in "${REPOS[@]}"; do
    update_repo "$WS/src/${entry%%:*}" "${entry%%:*}" "${entry#*:}"
done

$SKIP_BUILD && { step "Repos updated (--skip-build)"; exit 0; }

# --- ROS dependencies (e.g. PCL for stonefish_ros2) --------------------------------------

step "Installing the ROS dependencies of the simulator packages"
command -v rosdep >/dev/null || run sudo apt-get install -y python3-rosdep
[[ -d /etc/ros/rosdep/sources.list.d ]] || run sudo rosdep init
run rosdep update -q
(
    set +u # ROS setup files use unset variables
    source /opt/ros/humble/setup.bash
    run rosdep install -y --ignore-src --rosdistro humble --from-paths \
        "$WS/src/stonefish_ros2" "$WS/src/vortex-stonefish-sim" "$WS/src/vortex-msgs" "$WS/src/vortex-utils"
)

# --- Stonefish ---------------------------------------------------------------------------

CMAKE_ARGS=()
if [[ "$PREFIX" != /usr/local ]]; then
    PREFIX="$(mkdir -p "$PREFIX" && cd "$PREFIX" && pwd)"
    CMAKE_ARGS=(-DStonefish_DIR="$PREFIX/lib/cmake/Stonefish"
                -DCMAKE_INSTALL_RPATH="$PREFIX/lib" -DCMAKE_BUILD_RPATH="$PREFIX/lib")
fi

installed_version() {
    grep -m1 'set(PACKAGE_VERSION ' "$PREFIX/lib/cmake/Stonefish/StonefishConfigVersion.cmake" 2>/dev/null |
        sed -E 's/.*"(.*)".*/\1/' || true
}

SF_REBUILT=false
if ! $SKIP_SF; then
    stamp="$PREFIX/share/Stonefish/vortex_commit"
    if [[ "$(installed_version)" == "$STONEFISH_VERSION" && "$(cat "$stamp" 2>/dev/null)" == "$(git -C "$SF" rev-parse HEAD 2>/dev/null)" ]]; then
        step "Stonefish $STONEFISH_VERSION is installed and up to date in $PREFIX"
    else
        step "Building Stonefish into $PREFIX (takes a few minutes)"
        note "installed now: $(installed_version || true) ${SF_BEFORE:0:7}"
        SUDO=""
        [[ -w "$PREFIX" ]] || SUDO="sudo"
        run mkdir -p "$SF/build"
        run cmake -S "$SF" -B "$SF/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX"
        run nice -n 10 make -C "$SF/build" -j"$JOBS"
        run $SUDO make -C "$SF/build" install
        $DRY || git -C "$SF" rev-parse HEAD | $SUDO tee "$stamp" >/dev/null
        [[ "$PREFIX" == /usr/local ]] && run sudo ldconfig
        SF_REBUILT=true
    fi
fi

# --- Workspace ---------------------------------------------------------------------------

step "Building the simulator packages in $WS"
# stonefish_ros2 caches where Stonefish was found: start it from scratch after a new Stonefish
if $SF_REBUILT || ! grep -qsxE "Stonefish_DIR:[A-Z]+=$PREFIX/lib/cmake/Stonefish" "$WS/build/stonefish_ros2/CMakeCache.txt"; then
    note "clean build of stonefish_ros2"
    run rm -rf "$WS/build/stonefish_ros2" "$WS/install/stonefish_ros2"
fi
# A Stonefish in src/ gets built by colcon into install/Stonefish, which comes before $PREFIX:
# remove that copy so stonefish_ros2 uses the one installed above
if [[ -d "$WS/install/Stonefish" ]]; then
    note "removing the workspace's own Stonefish build (install/Stonefish)"
    run rm -rf "$WS/build/Stonefish" "$WS/install/Stonefish"
fi
(
    cd "$WS"
    set +u # ROS setup files use unset variables
    source /opt/ros/humble/setup.bash
    export MAKEFLAGS="-j$JOBS"
    run nice -n 10 colcon build --symlink-install --parallel-workers "$JOBS" \
        --packages-up-to stonefish_sim stonefish_sim_interface \
        ${CMAKE_ARGS[@]:+--cmake-args "${CMAKE_ARGS[@]}"}
)

step "Done"
cat <<EOF
    source $WS/install/setup.bash
    ros2 launch stonefish_sim vortex_sim_launch.py scenario:=robosub [drone:=orca] [spawn:=slalom]

    In the simulator window: Tab = scene editor (Q/W/E, Ctrl+S saves), "Debug view" for the waypoint
    and the landmark server's frames. Run this script again to update.
EOF
