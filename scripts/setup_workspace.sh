#!/usr/bin/env bash
set -euo pipefail

if [[ ${BASH_VERSINFO[0]} -lt 4 ]]; then
  echo "This script requires bash >= 4" >&2
  exit 1
fi

WORKSPACE=${1:-$HOME/mono_hydra_ws}
SRC_DIR="$WORKSPACE/src"

mkdir -p "$SRC_DIR"

clone_checkout() {
  local target=$1 url=$2 ref=$3
  if [[ ! -d "$SRC_DIR/$target" ]]; then
    git clone "$url" "$SRC_DIR/$target"
  fi
  pushd "$SRC_DIR/$target" >/dev/null
  git fetch --all --tags
  git checkout "$ref"
  popd >/dev/null
}

declare -A REPOS=(
  [catkin_simple]="https://github.com/catkin/catkin_simple.git 0e62848b12da76c8cc58a1add42b4f894d1ac21"
  [cmake_external_project_catkin]="https://github.com/zurich-eye/cmake_external_project_catkin.git 7c2ab9a04c400b74914abe773c1e27e5b195ce9f"
  [config_utilities]="https://github.com/MIT-SPARK/config_utilities.git 2b27c4d543a23b7d4c3300eb84c4ecc6864424b8"
  [dbow2_catkin]="https://github.com/MIT-SPARK/dbow2_catkin.git 210b907287deeb0f7133cda6da375a970e098fa5"
  [disparity_image_proc]="https://github.com/MIT-SPARK/disparity_image_proc.git c35c2ff6da31eb5ceceb326e4997a7edf8589db0"
  [gflags_catkin]="https://github.com/ethz-asl/gflags_catkin.git fc38fc525f7d48881aebb27a7b9978453556bbd4"
  [glog_catkin]="https://github.com/ethz-asl/glog_catkin.git 40a9edadd15c59f8b57dc947d0135b0a007ea10b"
  [gtsam]="https://github.com/borglab/gtsam.git c4184e192b4605303cc0b0d51129e470eb4b4ed1"
  [hydra]="https://github.com/MIT-SPARK/Hydra.git 0dd9653136d2f1f31a8a205d0e69a78a738d4403"
  [hydra_ros]="https://github.com/MIT-SPARK/Hydra-ROS.git 2123b67872aa53029ddd0e0d84ecf5e8d1907e92"
  [kimera_pgmo]="https://github.com/MIT-SPARK/Kimera-PGMO.git 31cc9cdc3791c89f766277742e9d1cf5401f6cea"
  [kimera_rpgo]="https://github.com/MIT-SPARK/Kimera-RPGO.git d28b4df0570d642a2bb00e511344ce1110f87519"
  [kimera_rviz_markers]="https://github.com/MIT-SPARK/kimera_rviz_markers.git f342baf99fa55f9351795785bea1262ed8b451db"
  [mesh_rviz_plugins]="https://github.com/MIT-SPARK/mesh_rviz_plugins.git 2ad232270b5728522fd504d5c337ed5e43bf46cf"
  [m2h]="https://github.com/UAV-Centre-ITC/M2H.git main"
  [mono_hydra]="https://github.com/UAV-Centre-ITC/mono_hydra.git main"
  [mono_hydra_vio]="https://github.com/BavanthaU/mono_hydra_vio.git main"
  [mono_hydra_vio_ros]="https://github.com/BavanthaU/mono_hydra_vio_ros.git main"
  [opengv]="https://github.com/laurentkneip/opengv.git 91f4b19c73450833a40e463ad3648aae80b3a7f3"
  [opengv_catkin]="https://github.com/MIT-SPARK/opengv_catkin.git b2cf789eb1dff2cc57ac65c8919cb8030807f4c5"
  [pose_graph_tools]="https://github.com/MIT-SPARK/pose_graph_tools.git 4f8c0b5008504246b8e7d2ca74e21bfbf180a245"
  [rvio2_mono]="https://github.com/BavanthaU/rvio2_mono.git 70e9ee7225a1e8c6201a0dc482f011371182e0e7"
  [spark_dsg]="https://github.com/MIT-SPARK/Spark-DSG.git 48405e85924e8f315fc775ee51c1980542374cd4"
  [spatial_hash]="https://github.com/MIT-SPARK/Spatial-Hash.git bf592f26d84beca96e3ddc295ee1cf5b7341dee5"
  [teaser_plusplus]="https://github.com/MIT-SPARK/TEASER-plusplus.git 9ca20d9b52fcb631e7f8c9e3cc55c5ba131cc4e6"
)

for entry in "${!REPOS[@]}"; do
  read -r url ref <<< "${REPOS[$entry]}"
  echo "=== $entry"
  clone_checkout "$entry" "$url" "$ref"
done

# Ensure Kimera-PGMO launch files reference the renamed mono_hydra_vio_ros package.
PGMO_DIR="$SRC_DIR/kimera_pgmo"
if [[ -d "$PGMO_DIR" ]]; then
  python3 - "$PGMO_DIR" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
for path in root.rglob("*"):
    if not path.is_file():
        continue
    try:
        text = path.read_text()
    except UnicodeDecodeError:
        continue
    if "kimera_vio_ros" not in text:
        continue
    new_text = text.replace("kimera_vio_ros", "mono_hydra_vio_ros")
    if new_text != text:
        path.write_text(new_text)
PY
fi

# Patch upstream TEASER++ to add missing std:: qualifiers and includes on older commits.
TEASER_GRAPH_CC="$SRC_DIR/teaser_plusplus/teaser/src/graph.cc"
if [[ -f "$TEASER_GRAPH_CC" ]]; then
  python3 - "$TEASER_GRAPH_CC" <<'PY'
from pathlib import Path
import re
import sys
path = Path(sys.argv[1])
text = path.read_text()
if "#include <vector>" not in text:
    anchor = '#include "teaser/graph.h"'
    if anchor in text:
        text = text.replace(anchor, '#include <vector>\n\n' + anchor, 1)
    else:
        text = '#include <vector>\n' + text
text = re.sub(r'(?<!std::)vector<', 'std::vector<', text)
path.write_text(text)
PY
fi

TEASER_GRAPH_TEST="$SRC_DIR/teaser_plusplus/test/teaser/graph-test.cc"
if [[ -f "$TEASER_GRAPH_TEST" ]]; then
  python3 - "$TEASER_GRAPH_TEST" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
text = path.read_text()

if "#include <vector>" not in text:
    anchor = '#include <map>'
    if anchor in text:
        text = text.replace(anchor, anchor + '\n#include <vector>', 1)
    else:
        text = '#include <vector>\n' + text

signature = 'void printAdjMatirx(const std::vector<std::vector<bool>>& adj, int nodes)'
replacement = 'template <typename BoolLike>\nvoid printAdjMatirx(const std::vector<std::vector<BoolLike>>& adj, int nodes)'
if signature in text:
    text = text.replace(signature, replacement, 1)

text = re.sub(r'(?<!std::)vector<', 'std::vector<', text)
path.write_text(text)
PY
fi

if command -v rosdep >/dev/null; then
  rosdep install --from-paths "$SRC_DIR" --ignore-src -r -y
fi

cat <<'EOM'
Workspace prepared.
Remember to run:
  source /opt/ros/noetic/setup.bash
  cd "$WORKSPACE"
  catkin config --extend /opt/ros/noetic --merge-devel \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF
  catkin build
EOM
