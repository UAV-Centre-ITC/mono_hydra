# mono_hydra


- **Mono-Hydra** – [*Mono-Hydra: Real-time 3D scene graph construction from monocular camera input with IMU*](https://arxiv.org/abs/2308.05515)
- **M2H** – [*M2H: Multi-Task Learning with Efficient Window-Based Cross-Task Attention for Monocular Spatial Perception*](https://arxiv.org/abs/2510.17363)
  
## Mono-Hydra flow chart
<img src="https://github.com/UAV-Centre-ITC/mono_hydra/blob/main/doc/media/Slide1.jpg">

[Poster](<doc/media/M2H_IROS25_A4.pdf>)

mono_hydra is the **central workspace** for our monocular spatial perception
stack. It aggregates:

- **M2H** multi-task perception (semantics + depth + normals + edges).
- **mono_hydra_vio** (Kimera-VIO fork) and **mono_hydra_vio_ros** for
  SuperPoint-based monocular/stereo VIO.
- **rvio2_mono** (fork of R-VIO2) for robocentric VIO without loop closing.
- Dataset launchers, TFs, RViz layouts, and helper scripts that glue the
  perception and VIO pipelines together.

## Installing mono_hydra

### Prerequisites

- Ubuntu 20.04 + ROS Noetic with `python3-catkin-tools` installed.
- `git` and `git-lfs` (`sudo apt install git git-lfs`).
- GPU drivers/CUDA if you plan to use the SuperPoint (ONNX Runtime) pipeline.

### 1. Clone the curated workspace

The Mono-Hydra stack depends on specific forks of Kimera-VIO and its ROS wrapper.
Hydra has seen many revisions over the past few years, so please use the setup
script below to confirm the entire workspace still builds end to end. I last
tested the script on Ubuntu 20.04 LTS with ROS Noetic. At the moment I cannot
take on feature requests; if you run into a bug, please open an issue and I will
do my best to address it promptly.
Use the helper script bundled in this repository to fetch the exact revisions.
You can either run it directly from GitHub (handy for first-time setup) or from
the checked-out repo.

```bash
# Optional: choose a custom workspace path
export MONO_WS=$HOME/mono_hydra_ws

# From anywhere
curl -fsSL https://raw.githubusercontent.com/UAV-Centre-ITC/mono_hydra/main/scripts/setup_workspace.sh \
  | bash -s -- "${MONO_WS:-$HOME/mono_hydra_ws}"

# or, once you have cloned mono_hydra locally:
#   cd /path/to/mono_hydra
#   ./scripts/setup_workspace.sh "${MONO_WS:-$HOME/mono_hydra_ws}"
```

The script clones:

- `mono_hydra` (this repository) from `UAV-Centre-ITC/mono_hydra`
- `m2h` from `UAV-Centre-ITC/M2H`
- `mono_hydra_vio` and `mono_hydra_vio_ros` from `github.com/BavanthaU`
- `rvio2_mono` fork with Mono-Hydra automation tweaks
- All upstream dependencies (Hydra, Spark-DSG, Kimera-RPGO/PGMO, gtsam, etc.)
  pinned to known-good commits.

After the script completes, source ROS and build:

```bash
source /opt/ros/noetic/setup.bash
cd "${MONO_WS:-$HOME/mono_hydra_ws}"
catkin config --extend /opt/ros/noetic --merge-devel \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DGTSAM_TANGENT_PREINTEGRATION=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DOPENGV_BUILD_WITH_MARCH_NATIVE=OFF
catkin build
```

> The script is also available locally under `mono_hydra/scripts/setup_workspace.sh`
> once the repository is cloned. Re-running the script is safe: it will `git
> fetch` and checkout the pinned commits if the repos already exist.

## Included pipelines

- **M2H perception** – launch via `roslaunch m2h m2h.launch`.
- **Kimera-based VIO** – `roslaunch mono_hydra hydra_v2_d435i.launch`
  (or other dataset-specific launchers under `mono_hydra/launch`). SuperPoint
  support lives in the `RealSense_RGBD_sp` and `ZEDXMono_RGBD` parameter sets.
- **rvio2_mono robocentric VIO (based on R-VIO2)** – `roslaunch rvio2 realsense.launch`
  for the RealSense-driven monocular workflow.

> **Sensor note:** mono_hydra has been validated on RealSense RGB + IMU and ZED X one
> rigs. The released M2H weights were not trained on wide-FOV RGB imagery, so
> the RealSense configuration currently yields the best perception quality.

## How to run Mono Hydra with ITC building dataset

### Download data 
Download the 2nd-floor loop of the old ITC building from [this SURFdrive archive](https://surfdrive.surf.nl/s/baJM4fj3DaWwg3T). Unzip the file and place the extracted ROS bag (e.g., `ITC_2nd_floor_full_loop.bag`).

### Quickstart (after a clean workspace build)
Open four terminals and launch the stack in order:

> **Feature tracker selection:**  
> - GFTT tracker: `roslaunch mono_hydra_vio_ros mono_hydra_vio_ros_realsense_rgbd.launch online:=true viz_type:=1 use_lcd:=true lcd_no_optimize:=true`  
> - SuperPoint tracker: use the `_sp` launch shown in step 2 below.
> 
> Kimera-based monocular VIO will still propose loop closure detections in these launch files, but the global optimization is handled downstream by mono_hydra.
```
roslaunch mono_hydra hydra_v2_d435i.launch use_gt_frame:=false
```
```
roslaunch mono_hydra_vio_ros mono_hydra_vio_ros_realsense_rgbd_sp.launch online:=true viz_type:=1 use_lcd:=true lcd_no_optimize:=true
```
```
roslaunch m2h m2h.launch
```
```
rosbag play /path/to/ITC_2nd_floor_full_loop.bag --clock --pause /tf:=/tf_ignore /tf_static:=/tf_static_ignore
```

> **Logging tip:** mono_hydra_vio_ros writes frontend/backend CSV logs under
> `mono_hydra_vio_ros/output_logs/<config_name>/`. If you add a new dataset
> configuration, create the matching subdirectory first (e.g.,
> `output_logs/RealSense_RGBD_sp/`) so logging succeeds.

## Attribution & citations

mono_hydra stands on the shoulders of the MIT-SPARK community. Hydra
([github.com/MIT-SPARK/Hydra](https://github.com/MIT-SPARK/Hydra)) and the
Kimera family ([github.com/MIT-SPARK/Kimera](https://github.com/MIT-SPARK/Kimera))
provide the spatial scene graph, factor graph optimization, and VIO foundations
that make this monocular workspace possible. If mono_hydra helps your research
or deployment, please cite the Hydra and Kimera papers alongside the Mono-Hydra
and M2H publications linked above.

### Results
**3D Mapping Test (ITC dataset) with M2H framework**

| Model | 2nd Floor ME (m) ↓ | 2nd Floor SD (m) ↓ | 3rd Floor ME (m) ↓ | 3rd Floor SD (m) ↓ | FPS ↑ |
| --- | --- | --- | --- | --- | --- |
| DistDepth[36]+HRNet[37] | 0.19 | 0.18 | 0.21 | 0.16 | 15 |
| MTMamba++ [11] | 0.21 | 0.22 | 0.18 | 0.19 | 18 |
| M2H-small (ours) | 0.16 | 0.18 | 0.15 | 0.17 | 42 |
| **M2H (ours)** | **0.11** | **0.14** | **0.10** | **0.13** | **30** |

<img src="https://github.com/UAV-Centre-ITC/mono_hydra/blob/main/doc/media/2nd_floor.jpg" width="80%" height="80%">

*2nd floor loop in the old ITC building (>100 m).*




### Citations
If you use this repository, please cite the following papers:

- **Mono-Hydra core pipeline**
  ```bibtex
  @article{Udugama_2023,
     title={MONO-HYDRA: REAL-TIME 3D SCENE GRAPH CONSTRUCTION FROM MONOCULAR CAMERA INPUT WITH IMU},
     volume={X-1/W1-2023},
     ISSN={2194-9050},
     url={http://dx.doi.org/10.5194/isprs-annals-X-1-W1-2023-439-2023},
     DOI={10.5194/isprs-annals-x-1-w1-2023-439-2023},
     journal={ISPRS Annals of the Photogrammetry, Remote Sensing and Spatial Information Sciences},
     publisher={Copernicus GmbH},
     author={Udugama, U. V. B. L. and Vosselman, G. and Nex, F.},
     year={2023},
     month=dec,
     pages={439--445}
  }
  ```

- **M2H multi-task perception**
  ```bibtex
  @article{udugama2025m2hmultitasklearningefficient,
     title={Multi-Task Learning with Efficient Window-Based Cross-Task Attention for Monocular Spatial Perception},
     author={Udugama, U. V. B. L. and Vosselman, G. and Nex, F.},
     journal={arXiv preprint arXiv:2510.17363},
     year={2025}
  }
  ```
