# Mono_Hydra
<img src="https://github.com/UAV-Centre-ITC/Mono_Hydra/blob/main/doc/media/illustration.png"  width="80%" height="80%">

[Poster](<doc/media/PSB Poster A3 gen.pdf>)
## Mono-Hydra flow chart
<img src="https://github.com/UAV-Centre-ITC/Mono_Hydra/blob/main/doc/media/flow.png"  width="50%" height="20%">

Mono_Hydra is the **central workspace** for our monocular spatial perception
stack. It aggregates:

- **M2H** multi-task perception (semantics + depth + normals + edges).
- **mono_hydra_vio** (Kimera-VIO fork) and **mono_hydra_vio_ros** for
  SuperPoint-based monocular/stereo VIO.
- **rvio2_mono** (fork of R-VIO2) for robocentric VIO without loop closing.
- Dataset launchers, TFs, RViz layouts, and helper scripts that glue the
  perception and VIO pipelines together.

## Installing Mono_Hydra

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
  @article{Udugama_2025_M2H,
     title={Multi-Task Learning with Efficient Window-Based Cross-Task Attention for Monocular Spatial Perception},
     author={Udugama, U. V. B. L. and Vosselman, G. and Nex, F.},
     journal={arXiv preprint arXiv:2510.17363},
     year={2025}
  }
  ```



### Prerequisites

- Ubuntu 20.04 + ROS Noetic with `python3-catkin-tools` installed.
- `git` and `git-lfs` (`sudo apt install git git-lfs`).
- GPU drivers/CUDA if you plan to use the SuperPoint (ONNX Runtime) pipeline.

### 1. Clone the curated workspace

The Mono-Hydra stack depends on specific forks of Kimera-VIO and its ROS wrapper.
Use the helper script bundled in this repository to fetch the exact revisions.
You can either run it directly from GitHub (handy for first-time setup) or from
the checked-out repo.

```bash
# Optional: choose a custom workspace path
export MONO_WS=$HOME/mono_hydra_ws

# From anywhere
curl -fsSL https://raw.githubusercontent.com/UAV-Centre-ITC/Mono_Hydra/main/scripts/setup_workspace.sh \
  | bash -s -- "${MONO_WS:-$HOME/mono_hydra_ws}"

# or, once you have cloned Mono_Hydra locally:
#   cd /path/to/Mono_Hydra
#   ./scripts/setup_workspace.sh "${MONO_WS:-$HOME/mono_hydra_ws}"
```

The script clones:

- `mono_hydra` (this repository) from `UAV-Centre-ITC/Mono_Hydra`
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

### 2. (Optional) Install Hydra manually
If you prefer manual setup or want to inspect Hydra independently, follow the
instructions at https://github.com/MIT-SPARK/Hydra/
before running the workspace script.

## Included pipelines

- **M2H perception** – launch via `roslaunch m2h m2h.launch`.
- **Kimera-based VIO** – `roslaunch mono_hydra hydra_v2_d435i.launch`
  (or other dataset-specific launchers under `mono_hydra/launch`). SuperPoint
  support lives in the `RealSense_RGBD_sp` and `ZEDXMono_RGBD` parameter sets.
- **rvio2_mono robocentric VIO (based on R-VIO2)** – `roslaunch rvio2 realsense.launch`
  for the RealSense-driven monocular workflow.

## How to run Mono Hydra with ITC building dataset

### Download data 
TODO

### ROS commands, Each in new terminal
```
roslaunch mono_hydra real_cam_hydra.launch start_visualizer:=true
```
**HRNetV2 trained on ADE20k**
```
roslaunch semantic_segmentation_ros semantic_segmentation.launch
```

**RIVO2 based odometry prediction node**
```
roslaunch rvio2 realsense.launch
```

**DistDepth model based depth prediction**
```
roslaunch depth_distdepth dist_depth.launch
```

**or Lite-Mono based depth prediction**
```
roslaunch depth_lite_mono lite_mono.launch
```

**Play the downloaded ROS bag file**
```
rosbag play office_3.bag --clock --pause /camera/color/image_raw:=/cam0/image_raw  /camera/imu:=/imu0 /tf:=/tf_ignore /tf_static:=/tf_static_ignore -r 0.1
```


## Results

| Depth prediction network |  2nd floor	    |   3rd floor     |
|:-------------------------|:--------------:|:---------------:|
|                          | ME (m)   \|  SD (m)	|   ME (m)    \|  SD (m)|
| DistDepth	               | 0.1979   \|  0.1810	|   0.2128    \|	0.1645|
| Lite-Mono	               | 0.3818   \|  0.2692	|   0.3617    \|  0.2468|

#### 2nd to 3rd floor Scene graph at the ITC building
<img src="https://github.com/UAV-Centre-ITC/Mono_Hydra/blob/main/doc/media/mh_itc_2nd_3rd.png"  width="60%" height="60%">
