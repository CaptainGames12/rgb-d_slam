# RGB-D SLAM Evaluation Using ScanNet Dataset and ROS 2

> Integrated CPS Project I & II (IP5.b)

A comparative evaluation of multiple **RGB-D SLAM (Simultaneous Localization and Mapping)** approaches integrated with **ROS 2** and tested using the **ScanNet dataset** for trajectory estimation and performance analysis.

---

## Project Overview

This project focuses on implementing, integrating, and evaluating different **RGB-D SLAM algorithms** within the **ROS 2 ecosystem** using pre-recorded sequences from the **ScanNet dataset**.

Rather than relying on live sensor acquisition, the project utilizes synchronized **RGB images, depth images, camera calibration data, and ground-truth poses** provided by ScanNet to simulate real-world SLAM scenarios and benchmark algorithm performance.

The primary objective is to estimate camera trajectories and compare them against the dataset’s ground truth using established trajectory evaluation metrics.

---

## Team Members

| Member                   | Student ID | Contributions                                                                                                                                            |
| ------------------------ | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Volodymyr Mazan**      | m12553388  | Implemented Open3D-SLAM, attempted GLIM integration, project coordination, documentation, evaluation scripts, presentation, GitHub repository management |
| **Vignesh Suresh Menon** | m12550429  | Implemented small_gICP, contributed to documentation                                                                                                     |
| **Swayam Virmani**       | m12552861  | Implemented PCL_ICP, presentation preparation, documentation                                                                                             |

### Supervisor

**Christian Rauch**

---

# Introduction

**Simultaneous Localization and Mapping (SLAM)** is one of the fundamental challenges in robotics and computer vision.

SLAM enables a system to:

* Estimate the position of a moving sensor (**Localization**)
* Construct a representation of the surrounding environment (**Mapping**)


In this project, multiple RGB-D SLAM pipelines were integrated into **ROS 2** and evaluated using benchmark datasets.

The estimated trajectories produced by each implementation were compared against the **ground-truth camera poses** provided by ScanNet.

---

# Problem Statement

The project aims to estimate the trajectory of a moving **RGB-D camera** using only the information available within the **ScanNet dataset**.

Each implemented SLAM algorithm processes RGB-D frames and outputs an estimated camera trajectory.

The generated trajectories are then compared against ground-truth poses using quantitative evaluation metrics.

---

## Evaluation Metrics

The performance of each SLAM implementation is evaluated using:

* **ATE (Absolute Trajectory Error)**
* **RMSE (Root Mean Square Error)**
* **Maximum Trajectory Error**
* **Standard Deviation of Error**

These metrics provide insight into:

* Localization precision
* Trajectory consistency
* Robustness across sequences
* Runtime vs accuracy trade-offs

---

# Project Objectives

* Install and configure **ROS 2** on Ubuntu
* Investigate existing **RGB-D SLAM approaches**
* Integrate selected SLAM frameworks into ROS 2
* Process RGB-D streams from ScanNet
* Estimate camera trajectories
* Compare estimated trajectories with ground truth
* Evaluate performance using trajectory metrics
* Analyse parameter influence on accuracy and runtime
* Compare different SLAM methods

---

## Expected Workflow

```text
ScanNet Dataset
      │
      ▼
RGB + Depth Stream
      │
      ▼
ROS 2 Integration
      │
      ▼
SLAM Algorithm
      │
      ▼
Trajectory Estimation
      │
      ▼
Ground Truth Comparison
      │
      ▼
Performance Evaluation
```

---

> 🚀 More sections coming: Architecture • Installation • Dataset Setup • Algorithms • Results • Visualizations • Conclusions

# 3. Background

## 3.1 RGB-D Cameras

RGB-D cameras capture both **visual appearance (RGB)** and **depth information** for every pixel in a scene.

The RGB stream provides colour information, while the depth stream measures the distance between the camera and objects in the environment. Combining these two modalities enables reconstruction of a three-dimensional representation of the observed scene.

Using camera calibration parameters (intrinsic parameters), each pixel in the depth image can be projected into 3D space to generate a **point cloud representation**.

### RGB-D Camera Outputs

* RGB image (colour information)
* Depth image (distance information)

### Camera Intrinsic Parameters

The transformation from image coordinates to 3D coordinates depends on:

* **fx** — focal length in x-direction
* **fy** — focal length in y-direction
* **cx** — principal point (x-coordinate)
* **cy** — principal point (y-coordinate)

These parameters allow each depth measurement to be mapped into real-world coordinates.

---

### General RGB-D SLAM Pipeline

Although implementation details differ across SLAM algorithms, most follow a similar processing pipeline:

```text
RGB + Depth Input
        │
        ▼
Feature / Geometry Extraction
        │
        ▼
Frame-to-Frame Motion Estimation
        │
        ▼
Trajectory Update
        │
        ▼
Map Construction / Maintenance
```

Typical factors influencing SLAM performance include:

* Sensor quality
* Feature extraction quality
* Point cloud registration accuracy
* Optimization methods
* Algorithm parameter selection
* Computational efficiency

---

## 3.2 Dataset

The project uses the **ScanNet dataset**, a large-scale RGB-D benchmark designed for indoor scene understanding and evaluation.

Each ScanNet sequence provides synchronized sensor data and reference trajectories, making it suitable for quantitative SLAM benchmarking.

### Dataset Components

* RGB image sequences
* Depth image sequences
* Camera intrinsic parameters
* Ground-truth camera poses

The dataset is distributed in **`.sens`** format and replayed inside **ROS 2** using a custom **ScanNet Publisher** node.

---

### Evaluated Sequences

The following scenes were selected for evaluation:

| Scene ID     |
| ------------ |
| scene0378_02 |
| scene0378_00 |
| scene0050_01 |
| scene0011_01 |
| scene0011_00 |

Ground-truth trajectories provided by ScanNet allow direct comparison against estimated trajectories generated by SLAM algorithms.

---

## 3.3 System Architecture

The evaluation environment was standardized across all implementations.

### Development Environment

| Component        | Version      |
| ---------------- | ------------ |
| Operating System | Ubuntu 24.04 |
| Middleware       | ROS 2 Jazzy  |
| Dataset          | ScanNet      |

---

### Evaluation Pipeline

```text
ScanNet Dataset (.sens)
        │
        ▼
ScanNet Publisher (ROS 2)
        │
        ▼
RGB + Depth + Ground Truth Stream
        │
        ▼
Individual SLAM Method
        │
        ▼
Estimated Camera Trajectory
        │
        ▼
Trajectory Evaluation (evo)
```

Trajectory outputs were evaluated against the provided ground-truth poses using:

* Absolute Trajectory Error (ATE)
* Root Mean Square Error (RMSE)
* Maximum Trajectory Error
* Standard Deviation

These metrics were used to compare localization quality and overall algorithm performance across different SLAM approaches.

# 4. Methods

## 4.1 Method 1 — Open3D-SLAM

**Implemented by:** Volodymyr Mazan

### Method Overview

Open3D-SLAM is a SLAM framework built on top of **Open3D**, a library designed for processing 3D data, geometric computation, and visualization.

The framework estimates camera trajectories and constructs maps using **Generalized Iterative Closest Point (G-ICP)** for point cloud registration.

Since the original implementation was developed for older ROS distributions and is no longer actively maintained, it was adapted for **ROS 2 Jazzy** as part of this project. Additional modifications included introducing **Lua-based configuration support** and integrating the pipeline into the common evaluation framework.

---

### Generalized Iterative Closest Point (G-ICP)

Generalized ICP (G-ICP) extends traditional **Iterative Closest Point (ICP)** by introducing a probabilistic registration model for aligning point clouds.

Standard ICP estimates correspondences primarily through geometric proximity, which can lead to instability in environments containing noise, sparse geometry, or weak structural features.

G-ICP improves robustness by representing local point neighborhoods as **Gaussian distributions**, allowing registration to be formulated as a **surface-to-surface optimization problem** rather than simple point-to-point matching.

This approach improves alignment quality and provides greater stability for trajectory estimation.

---

### Registration Principle

Traditional registration approaches:

* **Point-to-Point ICP** → minimizes Euclidean distance between matched points
* **Point-to-Plane ICP** → minimizes distance relative to local surface normals
* **Generalized ICP** → models local geometry statistically for improved alignment

The result is typically more stable pose estimation in complex environments.

---

### Implementation Pipeline

```text id="87mxd1"
RGB-D Scan
    │
    ▼
addRangeScan()
    │
    ▼
Odometry Buffer
    │
    ▼
Odometry Worker
(Scan-to-Scan Registration)
    │
    ▼
Mapping Buffer
    │
    ▼
Mapping Worker
(Scan-to-Map Registration)
    │
    ▼
Loop Closure Detection
    │
    ▼
Pose Graph Optimization
    │
    ▼
Submap & Trajectory Correction
```

---

### Processing Stages

#### 1. Data Acquisition

Incoming RGB-D frames are received and converted into a suitable internal representation.

#### 2. Odometry Estimation

Consecutive scans are aligned using scan-to-scan registration to estimate incremental motion.

#### 3. Mapping

The estimated poses are accumulated and aligned against the existing environment map.

#### 4. Loop Closure

Previously visited locations are detected to reduce accumulated drift.

#### 5. Pose Graph Optimization

Global optimization adjusts previously estimated poses to improve trajectory consistency.

#### 6. Trajectory Refinement

Updated poses are propagated to generate corrected submaps and final trajectory outputs.

---

This method served as one of the trajectory estimation baselines and was evaluated using the same ScanNet scenes and trajectory metrics applied across all project implementations.



Hyperparameter Tuning

The Open3D-SLAM implementation was evaluated under different parameter configurations to analyse their influence on trajectory estimation quality and computational performance.

The following parameters were tuned during experimentation:

Voxel Size
Controls point cloud downsampling resolution. Smaller values preserve more geometric detail but increase computational cost.
Cropping Parameters
Restrict the processed spatial region to reduce unnecessary computations and improve runtime.
Buffer Size
Determines how many scans are retained during odometry and mapping, influencing registration stability and memory usage.

scene0010_00
<img width="1092" height="386" alt="image" src="https://github.com/user-attachments/assets/6233bb02-355d-45c7-8c53-7f8feaf40dc9" />

<img width="1025" height="356" alt="image" src="https://github.com/user-attachments/assets/08ab3267-cfaa-47bb-ae10-062bc91a2793" />

scene0378_02

<img width="1022" height="382" alt="image" src="https://github.com/user-attachments/assets/3fc5cafd-16e2-49f4-a24d-addc08a98fbb" />
<img width="1142" height="353" alt="image" src="https://github.com/user-attachments/assets/eabc4413-e071-40b1-95fb-3ad83e9def4b" />




### Usage

The implementation is separated into two components:

* `open3d_slam_ros` → ROS 2 wrapper responsible for integration and execution
* `open3d_slam_core` → Core SLAM implementation responsible for trajectory estimation

---

### Configuration

Hyperparameters can be modified through **Lua configuration files** located inside:

```bash
open3d_slam_core/include/
```

After changing parameters, rebuild the workspace:

```bash
colcon build --symlink-install
```

---

### Running Trajectory Estimation

Start trajectory generation using:

```bash
bash ./launch_open3d.zsh
```

The script processes the `.sens` files specified inside the launch script.

---

### Evaluation

To compare estimated trajectories against ground truth and generate plots:

```bash
bash ./eval.zsh
```

---

### Environment Setup

Before execution, source both ROS 2 and workspace environments:

```bash
source /opt/ros/jazzy/setup.zsh
source install/setup.zsh
```

---

### Dependencies

The project requires a **compiled C++ installation of Open3D** available on the system before building and running.



## 4.2 Method 2 — small_gicp

**Implemented by:** Vignesh Suresh Menon

### 4.2.1 Overview

This implementation integrates the **small_gicp** registration library into a ROS 2 pipeline for RGB-D visual odometry and trajectory estimation.

Instead of using a live RGB-D sensor, the node processes synchronized RGB-D sequences from the **ScanNet** dataset. For every incoming frame, the depth image is converted into a point cloud, registered with previous observations using **Generalized ICP (GICP)**, and accumulated into a complete camera trajectory.

The estimated trajectory is exported in **TUM format** and evaluated against the ScanNet ground-truth trajectory using standard SLAM metrics.

---

### Key Features

* ROS 2 Jazzy integration
* Point cloud generation from RGB-D depth images
* Generalized ICP registration using **small_gicp**
* Automatic trajectory export in TUM format
* Evaluation using ATE and RPE
* Diagnostic trajectory visualization

---

### Development Environment

| Component            | Details                         |
| -------------------- | ------------------------------- |
| Operating System     | Ubuntu 24.04                    |
| ROS Distribution     | ROS 2 Jazzy                     |
| Registration Library | small_gicp                      |
| Dataset              | ScanNet                         |
| Evaluation           | evo, custom Python scripts      |
| Supporting Libraries | Open3D, NumPy, SciPy, cv_bridge |

---

### 4.2.2 System Architecture

The implementation consists of two ROS 2 nodes.

#### ScanNet Publisher

The publisher reads `.sens` files and publishes:

* `/camera/depth/image_raw`
* `/camera/depth/camera_info`
* `/camera/color/image_raw`
* `/camera_pose`
* `/clock`

This simulates a live RGB-D camera while providing synchronized ground-truth poses.

#### SLAM Node

The SLAM node subscribes to the published topics and performs:

1. Depth image acquisition
2. Point cloud reconstruction
3. Point cloud registration
4. Pose estimation
5. Trajectory generation
6. TUM trajectory export

---

### Processing Pipeline

```text
Depth Image
      │
      ▼
Depth Filtering
      │
      ▼
Point Cloud Reconstruction
      │
      ▼
Voxel Downsampling
      │
      ▼
KdTree Construction
      │
      ▼
Covariance Estimation
      │
      ▼
Generalized ICP Registration
      │
      ▼
Pose Composition
      │
      ▼
Trajectory Export
```

---

### 4.2.3 Point Cloud Generation

Each depth frame is converted from millimetres to metres before invalid measurements are removed.

Using the camera intrinsic parameters (`fx`, `fy`, `cx`, `cy`), every valid depth pixel is back-projected into 3D space to generate a point cloud.

Frames containing too few valid points are discarded to avoid unstable registration.

---

### 4.2.4 Point Cloud Registration

Before registration, every point cloud undergoes:

* Voxel grid downsampling
* KdTree construction
* Local covariance estimation

The processed clouds are then aligned using **Generalized ICP (GICP)**.

Compared to classical ICP, GICP models local surface geometry using covariance matrices, resulting in more robust registration in noisy indoor environments.

---

### 4.2.5 Producer–Consumer Architecture

A significant challenge during development was that the publisher generated frames faster than the registration pipeline could process them.

To prevent frame loss, the implementation adopts a **producer–consumer architecture**:

* The ROS subscriber immediately stores incoming frames in a thread-safe queue.
* A dedicated worker thread processes frames sequentially.
* The queue is fully drained before shutdown, ensuring every frame is processed.

This design eliminates dropped frames while allowing computation to proceed independently of the publishing rate.

---

### 4.2.6 Parameter Tuning

Several registration parameters were optimized experimentally using **scene0000_00**.

| Parameter                       | Final Value |
| ------------------------------- | ----------: |
| Downsampling Resolution         |      0.05 m |
| Number of Neighbours            |          50 |
| Maximum Correspondence Distance |      0.20 m |
| Maximum Iterations              |          50 |
| Voxel Leaf Size                 |      0.05 m |
| Processing Threads              |           4 |

These settings provided the best trade-off between trajectory accuracy and computational performance.

> Parameter tuning plots should be inserted here.

---

### 4.2.7 Evaluation

Trajectory estimation was evaluated using:

* Absolute Trajectory Error (ATE)
* Relative Pose Error (RPE)

Estimated trajectories were exported in **TUM format** and compared with ScanNet ground-truth trajectories using both custom evaluation scripts and the **evo** toolkit.

Diagnostic visualizations include:

* 3D trajectory comparison
* 2D trajectory projections
* Position components
* Orientation plots
* Error distribution
* Error vectors

---

### Results

The implementation achieved **sub-0.5 m mean ATE** on the majority of evaluated ScanNet scenes.

| Scene        | Mean ATE (m) |
| ------------ | -----------: |
| scene0518_00 |    **0.151** |
| scene0378_00 |    **0.244** |
| scene0011_00 |        0.315 |
| scene0000_00 |        0.395 |
| scene0005_00 |        0.398 |
| scene0378_02 |        0.451 |
| scene0050_01 |        0.665 |
| scene0011_01 |        1.541 |

The strongest performance was achieved on **scene0518_00**, while **scene0011_01** proved the most challenging due to rapid motion and limited geometric constraints.

---

### Discussion

The scan-to-map approach demonstrated improved robustness compared to scan-to-scan registration by reducing long-term drift and enabling partial recovery after temporary registration failures.

Performance degradation was primarily observed in sequences containing:

* Rapid camera motion
* Low-overlap observations
* Repetitive indoor structures
* Feature-poor environments

Since the current implementation does not incorporate loop closure or pose graph optimization, accumulated drift cannot be corrected globally.

---

### 4.2.8 Reproducing the Results

#### Launch the ScanNet Publisher

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run scannet_publisher scannet_publisher \
  --ros-args -p file:=/path/to/SCENE.sens
```

#### Launch the SLAM Node

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run small_gicp_slam slam_node
```

#### Evaluate the Results

```bash
python3 evaluate_trajectory.py groundtruth.txt estimated.txt

python3 plot_trajectory.py groundtruth.txt estimated.txt output_directory/
```

Optional evaluation using **evo**:

```bash
evo_ape tum groundtruth_tum.txt estimated_tum.txt \
    --align --correct_scale
```


Additional implementation details and configuration notes are available inside the documentation files located in:

```bash
open3d_slam_ros/
```


## 4.3 Method 3 — PCL ICP RGB-D SLAM

**Implemented by:** Swayam Virmani

### 4.3.1 Overview

This implementation presents a complete **RGB-D SLAM** pipeline using the **Iterative Closest Point (ICP)** algorithm from the **Point Cloud Library (PCL)** for camera pose estimation.

The pipeline processes RGB-D sequences from the **ScanNet** dataset by converting depth images into point clouds, registering consecutive observations using ICP, and accumulating the estimated transformations to reconstruct the complete camera trajectory.

Unlike feature-based SLAM systems, this approach relies entirely on geometric registration. While computationally efficient and straightforward to implement, it does not incorporate loop closure or global optimization, making it susceptible to accumulated drift over long trajectories.

---

### Key Features

* RGB-D point cloud generation
* Voxel grid downsampling
* Point-to-point ICP registration using PCL
* Frame-to-frame pose estimation
* Automatic trajectory generation
* Trajectory evaluation using ATE
* Hyperparameter analysis

---

### 4.3.2 ICP Registration

The **Iterative Closest Point (ICP)** algorithm estimates the rigid transformation between two overlapping point clouds by minimizing the distance between corresponding points.

Each registration cycle consists of:

1. Finding nearest-neighbour correspondences.
2. Estimating the optimal rigid transformation.
3. Applying the transformation to the source cloud.
4. Repeating until convergence or the maximum number of iterations is reached.

The resulting transformation is accumulated with previous estimates to construct the full camera trajectory.

---

### 4.3.3 System Architecture

```text
ScanNet Dataset (.sens)
        │
        ▼
ScanNet Publisher (ROS 2)
        │
        ▼
Depth Image Stream
        │
        ▼
Point Cloud Generation
        │
        ▼
Voxel Grid Filtering
        │
        ▼
ICP Registration (PCL)
        │
        ▼
Pose Estimation
        │
        ▼
Trajectory Saving
        │
        ▼
Trajectory Evaluation
(ATE / RPE)
        │
        ▼
Plots & Metrics
```

**System Architecture Diagram**

<img width="380" height="296" alt="image" src="https://github.com/user-attachments/assets/4824fbeb-9916-438c-b0a4-b71383364c96" />


---

### 4.3.4 Installation & Dependencies

Install the required dependencies before building the project.

```bash
sudo apt install libpcl-dev
sudo apt install python3-pip

pip install numpy scipy matplotlib pandas
```

**Dependency Diagram**

<img width="240" height="236" alt="image" src="https://github.com/user-attachments/assets/3c6833cf-2bc1-4256-8632-d05d1de5d3d0" />


---

### 4.3.5 Running the Pipeline

The complete pipeline can be launched using the provided execution script.

bash run_pipeline.sh

The script automatically:

* Launches the ScanNet Publisher
* Starts the ICP SLAM node
* Records estimated trajectories
* Executes trajectory evaluation
* Generates plots and metrics
* Stores results in the corresponding scene directory

**Execution Workflow**

<img width="171" height="91" alt="image" src="https://github.com/user-attachments/assets/c39aa0fe-994a-412f-a313-65e7cf2296a1" />


---

### 4.3.6 Hyperparameter Tuning

To improve registration quality and computational performance, several ICP parameters were evaluated.

#### Voxel Grid Size

Voxel grid filtering reduces the number of points before registration.

* Smaller voxel sizes preserve more geometric detail.
* Larger voxel sizes improve runtime while reducing registration accuracy.

<img width="677" height="291" alt="image" src="https://github.com/user-attachments/assets/9873ef5d-f8c5-41b9-8e2e-8d2fcf9af126" />


---

#### Maximum ICP Iterations

Controls the maximum number of optimization iterations performed before convergence.

Higher values generally improve alignment accuracy but increase computation time.

<img width="614" height="278" alt="image" src="https://github.com/user-attachments/assets/f48c6ab0-e93f-46d8-a3ec-ba03b651ef51" />


---

#### Maximum Correspondence Distance

Defines the maximum distance allowed between two corresponding points during registration.

Lower thresholds reject incorrect matches, while larger thresholds improve robustness in scenes with larger inter-frame motion.

<img width="680" height="339" alt="image" src="https://github.com/user-attachments/assets/ead72bd2-cbd0-4799-ab35-e1ca8f4c1f1b" />


---

### 4.3.7 Results

The proposed pipeline was evaluated on multiple ScanNet indoor sequences.

Trajectory quality was assessed using:

* Absolute Trajectory Error (ATE)
* Trajectory visualization
* Error distribution analysis

---

#### Best Performing Scene — `scene0378_00`

This sequence achieved the lowest **Mean ATE (0.7132 m)**.

The estimated trajectory closely follows the ground truth for most of the sequence, demonstrating stable point cloud registration and consistent pose estimation.

**Trajectory Comparison**

<img width="654" height="545" alt="image" src="https://github.com/user-attachments/assets/5528e9ab-ea30-403f-b665-083f61af2cc4" />


**ATE Over Time**

<img width="637" height="450" alt="image" src="https://github.com/user-attachments/assets/3681fe10-7ba9-40c3-bc6f-879ae8451257" />


**Trajectory Error Heatmap**

<img width="622" height="255" alt="image" src="https://github.com/user-attachments/assets/48142803-05d8-4b03-9661-c1fd37e09fef" />



---

#### Worst Performing Scene — `scene0011_01`

This sequence recorded the highest **Mean ATE (3.3213 m)**.

The estimated trajectory gradually diverges from the ground truth as local registration errors accumulate over time. Since the implementation performs only frame-to-frame ICP registration, drift cannot be corrected without loop closure or global optimization.

**Trajectory Error Heatmap**

<img width="403" height="428" alt="image" src="https://github.com/user-attachments/assets/73563b4a-6487-490a-91f6-3a768d3ba2fe" />

The increasing concentration of high-error regions illustrates the accumulation of drift throughout the sequence.

---

#### Multi-Scene Evaluation

<img width="623" height="371" alt="image" src="https://github.com/user-attachments/assets/9efe47b2-b10d-40b5-b0f0-b7f1aecdcc4a" />

<img width="445" height="536" alt="image" src="https://github.com/user-attachments/assets/19b154a3-0c7f-4cdd-8748-b7be70657387" />

---

### 4.3.8 Analysis

The proposed PCL ICP implementation demonstrated reliable trajectory estimation in environments containing sufficient geometric structure and overlap between consecutive frames.

Among the evaluated sequences:

* **Best Scene:** `scene0378_00` (Mean ATE = **0.7132 m**)
* **Worst Scene:** `scene0011_01` (Mean ATE = **3.3213 m**)

Overall, the results show that classical ICP remains an effective baseline for RGB-D SLAM. However, trajectory drift increases over long sequences because the current implementation does not include loop closure, pose graph optimization, or global map correction.



# 5. Comparative Evaluation

The three RGB-D SLAM implementations were evaluated using the same ScanNet sequences and benchmarked using **Absolute Trajectory Error (ATE)**. Lower ATE values indicate more accurate trajectory estimation.

## Mean Absolute Trajectory Error (ATE)

| Scene        |    PCL ICP | small_gicp | Open3D-SLAM |
| ------------ | ---------: | ---------: | ----------: |
| scene0011_00 |     2.7223 | **0.3147** |      1.0392 |
| scene0011_01 |     3.3213 | **1.5410** |      1.7808 |
| scene0050_01 |     1.2858 |     0.6651 |  **0.4974** |
| scene0378_00 | **0.7132** |     0.2436 |      1.6160 |
| scene0378_02 |     0.8392 | **0.4508** |      0.7656 |

> *Overall comparison chart to be inserted here.*

---

## Performance Analysis

### small_gicp

The **small_gicp** implementation achieved the most consistent performance across the evaluated sequences. It produced the lowest trajectory errors in the majority of the tested scenes and maintained sub-meter localization accuracy in challenging indoor environments.

Its improved robustness can be attributed to:

* Generalized ICP (GICP) registration
* Covariance-based surface modeling
* Optimized point cloud preprocessing
* Scan-to-map registration strategy

---

### Open3D-SLAM

Open3D-SLAM generally provided intermediate performance between **small_gicp** and **PCL ICP**.

It achieved the **lowest mean ATE** on **scene0050_01 (0.4974 m)**, demonstrating strong performance in that environment. However, its accuracy decreased noticeably on **scene0378_00**, where it produced higher trajectory errors than the other evaluated methods.

---

### PCL ICP

The PCL ICP implementation served as a reliable baseline for geometric registration.

While it performed well in environments with strong geometric overlap, its frame-to-frame registration strategy accumulated drift over longer trajectories due to the absence of:

* Loop closure
* Pose graph optimization
* Global map refinement

Consequently, localization accuracy degraded in longer or more challenging sequences.

---

## Overall Comparison

Across the evaluated ScanNet scenes, **small_gicp** demonstrated the strongest overall performance by achieving the lowest trajectory errors in most sequences and maintaining more stable localization throughout longer trajectories.

Open3D-SLAM achieved competitive performance in several scenes and produced the best result on **scene0050_01**, while the PCL ICP implementation provided a solid baseline but exhibited greater accumulated drift because it relied solely on local frame-to-frame registration.

Overall, the experimental results indicate that **small_gicp** offers the best balance between robustness and trajectory accuracy among the three evaluated RGB-D SLAM approaches.



<img width="708" height="417" alt="image" src="https://github.com/user-attachments/assets/5f7849e0-7283-44ac-aa17-cd4ecc44f2bd" />

