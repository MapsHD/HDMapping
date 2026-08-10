!!! Recent benchmark results (28 LIO algorithms) [Bunker-DVI-Dataset-reg-1](https://github.com/MapsHD/benchmark-HDMapping-Orchestration/tree/Bunker-DVI-Dataset-reg-1)

# PROJECT ONBOARDING VIDEOS - OPENING MEASUREMENT SEASON 2026

[![MANDEYE-D](https://img.youtube.com/vi/ntF8kO8r8MM/default.jpg)](https://youtu.be/ntF8kO8r8MM)
[![MANDEYE-PRO](https://img.youtube.com/vi/Is4IvLiTCyw/default.jpg)](https://youtu.be/Is4IvLiTCyw)
[![USER FREINDLY SOFTWARE](https://img.youtube.com/vi/rJST0zhNVwc/default.jpg)](https://youtu.be/rJST0zhNVwc)

# Open-source (Windows, Linux and macOS), open hardware hand-held/wearable/mountable mobile mapping system for large-scale surveys:
This is open-source software for large-scale 3D mapping using an open-hardware hand-held/wearable/mountable measurement device available at https://github.com/JanuszBedkowski/mandeye_controller. 
We provide an end-to-end mobile mapping framework that does not require any installation, including:

- **HDMapping_LI0**: our implementation of LiDAR Inertial Odometry that outperforms the State of the Art [[movie]](https://youtu.be/UB-Hx7qgey8).
- **HDMapping_Pose_GRAPH_SLAM** to create city-level maps.
- **HDMapping_Georeferencing** (GNSS-RTK, Control Points, Ground Control Points, TLS, ALS).

# Workflow

![workflow](images/workflow.png)

# DOWNLOAD SOFTWARE USING THIS LINK -> [v0.103](https://github.com/MapsHD/HDMapping/releases/tag/v0.103)

If You are looking for end-to-end mobile mapping framework You can start from this project.
We provide a benchmark for other approaches: BIEVR-LIO (2026), SE3-LIO (2026), EllipseLIO(2026), D-LIO (2026), FORM (2026), Super-LIO (2026), DALI_SLAM (2025), lidar_odometry_ros_wrapper (2025), SuperOdometry (2021), mola_lidar_odometry (2025), RESPLE (2025), GenZ-ICP (2025), MM-LINS (2025), Voxel-SLAM (2024), NV-LIOM (2024), c3p-voxelmap (2024), I2EKF-LO (2024), LIO-EKF (2024), iG-LIO (2024), GLIM (2024), Point-LIO (2024), LOG-LIO2 (2024), DLIO (2023), SLICT (2023), RA-L, KISS-ICP (2023), Faster-LIO (2022), VoxelMap (2022), DLO (2022), CT-ICP (2021), FAST-LIO (2020), LOAM-Livox (2019), LeGO-LOAM (2018).
**Our goal is to find/develop/discover best end-to-end mobile mapping framework.**
To cite benchmark suite please use as follows:
```
@article{BEDKOWSKI2026102822,
title = {MapsHD: A benchmark suite for LiDAR odometry frameworks},
journal = {SoftwareX},
volume = {35},
pages = {102822},
year = {2026},
issn = {2352-7110},
doi = {https://doi.org/10.1016/j.softx.2026.102822},
url = {https://www.sciencedirect.com/science/article/pii/S2352711026003146},
author = {Janusz Bȩdkowski and Michał Pełka and Karol Majek and Marcin Matecki and Adrian Radulescu and Charles Hamesse and Ethan Decleyn and Przemysław Lekston and Tomasz Owerko and Przemysław Kuras and Michał Ciszewski and Jakub Kolecki and Karolina Tomaszkiewicz and Łukasz Ambroziński and Joanna Koszyk and Bartosz Hyla and Karolina Pargieła and Anna Malczewska and Tomasz Lipecki and Artur Adamek and Bartosz Mitka and Klapa Przemysław and Pelagia Gawronek and Martin Mokros and Jozef Výboštok and Juliána Chudá and Michal Skladan and Carlos Cabo and Kim André Anstensen and Craciun Daniel-Marian and Antun Jakopec and Michal Wlasiuk and Kornel Mrozowski and Maksymilian Kulicki and Krzysztof Stereńczak and Oskar Bartosz and Jakub Markiewicz and Sławomir Łapiński and Adam Kostrzewa and Mariana Campos and Machi Zawidzki and Jacek Szklarski and Rami Faraj and Loris Redovniković and Jurica Jagetić and Samer Karam and Răzvan Dumbravă and Milosz Mielcarek and Grzegorz Krok and Michal Laszkowski and Jaroslaw Wajs and Jakub Chudziński},
keywords = {LiDAR odometry, LiDAR-inertial odometry, Benchmarking},
abstract = {This paper describes a software toolbox for LiDAR (Light Detection and Ranging) and LiDAR-Inertial Odometry qualitative and quantitative evaluation. We provide software as https://github.com/MapsHD organization with all necessary information at https://github.com/MapsHD/HDMapping. Our software contributions are a) ground truth data processing tool, b) dockerized state-of-the-art LO and LIO algorithms, c) multi-session data registration to common coordinate system, d) Absolute Pose Error (APE) and Relative Pose Error (RPE) metrics, e) import/export tools for easier 3D data handling and visualizing, e.g., in Cloud Compare software. This software is compatible with ROS1 (Robot Operating System) and ROS2 data formats. We show an example benchmark of LeGO-LOAM, LIO-SAM, FAST-LIO, DLO, VoxelMap, Faster-LIO, KISS-ICP, CT-ICP, SLICT, DLIO, GLIM, iG-LIO, LIO-EKF, I2EKF-LO, GenZ-ICP, RESPLE, odometry_ros_wrapper, Point-LIO, and LOAM-Livox algorithms. For all experiments we provide movies. The contribution of the paper is software-oriented LO/LIO algorithm benchmark suite. The novelty lies in the integration of multiple benchmarking steps into a unified framework, thus overall effort needed for qualitative and quantitative evaluation is reduced.}
}
```

# Supported LiDARs

![lidars](images/IMG_2561.jpeg)
We support LIVOX AVIA, HAP, MID360, Ouster OS0, OS1, OS2, OSDome, SICK multiScan100, HESAI JT16, HESAI JT128, HESAI XT, Robosense AIRY.
HESAI XT requires external IMU. We are going to support more LiDARs ASAP.

More information can be found here:

- The introductory paper is available here: https://www.sciencedirect.com/science/article/pii/S235271102300314X
- Sample data is available at https://github.com/MapsHD/OmniWarsawDataset
- VIDEO (how to build mobile mapping hardware) https://www.youtube.com/watch?v=BXBbuSJMFEo
- If you are a ROS user, please visit https://github.com/MapsHD/mandeye_to_bag to convert MANDEYE data to ROSBAG
- ROS2 wrapper for HDMapping LiDAR Inertial Odometry (HDMapping-LIO) https://github.com/MapsHD/HDMapping-LIO
- ROSCON 2024 workshop (sample data sets and more ...): https://michalpelka.github.io/RosCon2024_workshop/
- You can use it also for multi-view Terrestrial Laser Scanner Registration (Faro, Leica, Z+F, Riegl, etc...) https://www.sciencedirect.com/science/article/abs/pii/S0263224123007637
- Info for Windows users: please use the latest release https://github.com/MapsHD/HDMapping/releases
- Contact email: januszbedkowski@gmail.com

# MANDEYE is action

[![MANDEYE-K9](https://img.youtube.com/vi/7a_o7ACH0-M/default.jpg)](https://youtu.be/7a_o7ACH0-M)
[![MANDEYE-MR (Caver)](https://img.youtube.com/vi/Bu9kDF5y39s/default.jpg)](https://youtu.be/Bu9kDF5y39s)
[![MANDEYE-MR (Precise Forestry)](https://img.youtube.com/vi/i6Xg_vPuqrY/default.jpg)](https://youtu.be/i6Xg_vPuqrY)

Our MANDEYE is designed for freedom in motion. You can also climb and crawl in most challenging scenarios.

# Compatible community projects

- Handheld Setup for Recording with Mid360 Lidar and Camera https://github.com/RomanStadlhuber/livo-handheld
- Lidar odometry for HDMapping project utilizing KISS-ICP https://github.com/michalpelka/kiss-lidarodometry


    
# To cite this work please use as follows:

Mobile mapping system: hardware and software are elaborated in the following paper:

```
@article{BEDKOWSKI2024101618,
     title = {Open source, open hardware hand-held mobile mapping system for large scale surveys},
     journal = {SoftwareX},
     volume = {25},
     pages = {101618},
     year = {2024},
     issn = {2352-7110},
     doi = {https://doi.org/10.1016/j.softx.2023.101618},
     url = {https://www.sciencedirect.com/science/article/pii/S235271102300314X},
     author = {Janusz Będkowski},
     keywords = {A mobile mapping, Lidar odometry, Loop closure, Iterative closest point, Data registration, SLAM},
     abstract = {This paper presents open-source software for large-scale 3D mapping using an open-hardware hand-held measurement device. This work is dedicated to educational and research purposes. This software is composed of three components: LIDAR odometry, single-session refinement, and multi-session refinement. Data refinement uses a pose-graph loop closure technique and an Iterative Closest Point algorithm to minimize the error of the edge. The results are 3D point clouds in LAZ data format (compressed LAS - LIDAR Aerial Survey). It was tested in many real-world scenarios/applications: city-level 3D mapping, culture heritage, creating ground truth data for mobile robots, precise forestry, and large-scale indoor 3D mapping. This software can run on Linux and Windows machines, it does not incorporate GPU computing. It is advised to use at least 32 GB of RAM to cope with large data sets. The optimization framework is implemented from scratch using the Eigen library, thus there is no need to install any additional libraries such as Ceres, g2o, GTSAM, manif, Sophus, etc.}
}
```

Terrestrial Laser Scanner data registration is elaborated in following paper:

```
@article{BEDKOWSKI2023113199,
     title = {Benchmark of multi-view Terrestrial Laser Scanning Point Cloud data registration algorithms},
     journal = {Measurement},
     pages = {113199},
     year = {2023},
     issn = {0263-2241},
     doi = {https://doi.org/10.1016/j.measurement.2023.113199},
     url = {https://www.sciencedirect.com/science/article/pii/S0263224123007637},
     author = {Janusz Będkowski},
     keywords = {TLS, Point cloud, Open-source, Multi-view data registration, LiDAR data metrics, Robust loss function, Tait-Bryan angles, Quaternions, Rodrigues’ formula, Lie algebra, Rotation matrix parameterization},
     abstract = {This study addresses multi-view Terrestrial Laser Scanning Point Cloud data registration methods. Multiple rigid point cloud data registration is mandatory for aligning all scans into a common reference frame and it is still considered a challenge looking from a large-scale surveys point of view. The goal of this work is to support the development of cutting-edge registration methods in geoscience and mobile robotics domains. This work evaluates 3 data sets of a total 20 scenes available in the literature. This paper provides a novel open-source framework for multi-view Terrestrial Laser Scanning Point Cloud data registration benchmarks. The goal was to verify experimentally which registration variant can improve the open-source data looking from the quantitative and qualitative points of view. In particular, the following scanners provided measurement data: Z+F TLS Imager 5006i, Z+F TLS Imager 5010C, Leica ScanStation C5, Leica ScanStation C10, Leica P40 and Riegl VZ-400. The benchmark shows an impact of the metric e.g. point to point, point to projection onto a plane, plane to plane, etc..., rotation matrix parameterization (Tait-Bryan, quaternion, Rodrigues) and other implementation variations (e.g. multi-view Normal Distributions Transform, Pose Graph SLAM approach) onto the multi-view data registration accuracy and performance. An open-source project is created and it can be used for improving existing data sets reported in the literature, it is the added value of the presented research. The combination of metrics, rotation matrix parameterization, and optimization algorithms creates hundreds of possible approaches. It is shown that the chosen metric is a dominant factor in data registration. The rotation parameterization and other degrees of freedom of proposed variants are rather negligible compared with the chosen metric. Most of the proposed approaches improve registered reference data provided by other researchers. Only for 2 of 20 scenes it was not possible to provide significant improvement. The largest improvements are evident for large-scale scenes. The project is available and maintained at https://github.com/MapsHD/HDMapping.}
}
```

The **HDMapping** optimization framework is implemented from scratch https://github.com/JanuszBedkowski/observation_equations using the Eigen library, thus there is no need to install any additional libraries such as **Ceres**, **g2o**, **GTSAM**, **manif**, **Sophus**, etc.
More information can be found in:

```
@book{DBLP:series/cir/Bedkowski22,
  author       = {Janusz Bedkowski},
  title        = {Large-Scale Simultaneous Localization and Mapping},
  series       = {Cognitive Intelligence and Robotics},
  publisher    = {Springer},
  year         = {2022},
  url          = {https://doi.org/10.1007/978-981-19-1972-5},
  doi          = {10.1007/978-981-19-1972-5},
  isbn         = {978-981-19-1971-8},
  timestamp    = {Mon, 25 Jul 2022 08:41:19 +0200},
  biburl       = {https://dblp.org/rec/series/cir/Bedkowski22.bib},
  bibsource    = {dblp computer science bibliography, https://dblp.org}
}
```

# 0.101 VIDEOs

Make session from ALS

[![Make session from ALS](https://img.youtube.com/vi/k6nysHveoKU/default.jpg)](https://youtu.be/U7hYEQfdfDU)

Make cave map

[![Make cave map](https://img.youtube.com/vi/k6nysHveoKU/default.jpg)](https://youtu.be/srcf6qu7I08)

Using RTK GNSS / GPS to georeference project


[![Make cave map](https://img.youtube.com/vi/7ikkAKHmu0w/default.jpg)](https://youtu.be/7ikkAKHmu0w)
# 0.94 VIDEOs

IMU inclination prior

[![IMU inclination prior](https://img.youtube.com/vi/k6nysHveoKU/default.jpg)](https://youtu.be/k6nysHveoKU)

# Obsolete VIDEOs

Where to find, how to download HDMAPPING software and manual

[![Where to find, how to download HDMAPPING software and manual](https://img.youtube.com/vi/dHCOAeto9-M/default.jpg)](https://youtu.be/dHCOAeto9-M)

Air ground survey

[![survey - precise forestry application](https://img.youtube.com/vi/_nvq2un_lFc/default.jpg)](https://youtu.be/_nvq2un_lFc)

Air ground survey result

[![result of precise forestry application](https://img.youtube.com/vi/InqpiFrPizg/default.jpg)](https://youtu.be/InqpiFrPizg)

Data recorded with Mandeye, online data registration with FAST-LIO.

[![data recorded with Mandeye, online data registration with FAST-LIO](https://img.youtube.com/vi/u8siB0KLFLc/default.jpg)](https://youtu.be/u8siB0KLFLc)

How to build Mandeye DEV

[![how to build Mandeye DEV](https://img.youtube.com/vi/BXBbuSJMFEo/default.jpg)](https://youtu.be/BXBbuSJMFEo)

---

Instruction for precise forestry

1. Fast and fully automatic calculations in a single step

[![Fast and fully automatic calculations in single step](https://img.youtube.com/vi/08E41CPyyj4/default.jpg)](https://youtu.be/08E41CPyyj4)

2. hdmapping precise forestry intro

[![hdmapping precise forestry intro](https://img.youtube.com/vi/W2ZHNOdZsq4/default.jpg)](https://youtu.be/W2ZHNOdZsq4)

3. hdmapping percise forestry raw data inspection

[![hdmapping percise forestry raw data inspection](https://img.youtube.com/vi/WRmW2hi1Cug/default.jpg)](https://youtu.be/WRmW2hi1Cug)

4. hdmapping percise forestry lidar odometry STEP1

[![hdmapping percise forestry lidar odometry STEP1](https://img.youtube.com/vi/laPuPJRoj5U/default.jpg)](https://youtu.be/laPuPJRoj5U)

5. hdmapping precise forestry session inspection

[![hdmapping precise forestry session inspection](https://img.youtube.com/vi/rmLXZh7SQTs/default.jpg)](https://youtu.be/rmLXZh7SQTs)

6. hdmapping precise forestry session inspection

[![hdmapping precise forestry session inspection](https://img.youtube.com/vi/Biz_OA8x1Ek/default.jpg)](https://youtu.be/Biz_OA8x1Ek)

---

Nuclear Power Plant inspection PART 1

[![Nuclear Power Plant inspection PART 1](https://img.youtube.com/vi/bpXQYZkH8Sc/default.jpg)](https://youtu.be/bpXQYZkH8Sc)

Nuclear Power Plant inspection PART 2

[![Nuclear Power Plant inspection PART 2](https://img.youtube.com/vi/fJcuGw1RLO0/default.jpg)](https://youtu.be/fJcuGw1RLO0)

---

Cave surveys (climbing, crawling in caves with MANDEYE-MR)

[![Cave data processing](https://img.youtube.com/vi/4iq69c76eG8/default.jpg)](https://youtu.be/4iq69c76eG8)

---

Multilevel building: example of using new functionality in v0.75 - intersection

[![Multi level building](https://img.youtube.com/vi/XYIHKyaxQzo/default.jpg)](https://youtu.be/XYIHKyaxQzo)

---

MANDEYE Mission Recorder calibration

[![MANDEYE Mission Recorder calibration](https://img.youtube.com/vi/V9L0a6aqsJ8/default.jpg)](https://youtu.be/V9L0a6aqsJ8)

---

MANDEYE georeferencing to GPS (NMEA data stream)

[![MANDEYE georeferencing to GPS (NMEA data stream)](https://img.youtube.com/vi/FOWPGCgOKI4/default.jpg)](https://youtu.be/FOWPGCgOKI4)

Removing GPS inconsistency after MANDEYE georeferencing to GPS (NMEA data stream)

[![Removing GPS inconsistency after MANDEYE georeferencing to GPS (NMEA data stream)](https://img.youtube.com/vi/gcUV22D4YrY/default.jpg)](https://youtu.be/gcUV22D4YrY)

MANDEYE georeferencing to GCPs (ground control points)

[![MANDEYE georeferencing to GCPs (ground control points)](https://img.youtube.com/vi/iLlU7xzYDe0/default.jpg)](https://youtu.be/iLlU7xzYDe0)

MANDEYE georeferencing to CPs (control points)

[![MANDEYE georeferencing to CPs (control points)](https://img.youtube.com/vi/ogeAjzJvLeY/default.jpg)](https://youtu.be/ogeAjzJvLeY)

---

Manual coloring

[![MANDEYE manual coloring](https://img.youtube.com/vi/EUbAaJp-XmY/default.jpg)](https://youtu.be/EUbAaJp-XmY)

---

# Compatible commercial products

[![MANDEYE-PRO](https://img.youtube.com/vi/EBUxUBWKIco/default.jpg)](https://youtu.be/EBUxUBWKIco)

---



![mandeye](images/softwareX1.png)

Mobile mapping systems is based on LiVOX MID360 - laser scanner with non repetetive scanning pattern.
Specification is available at https://www.livoxtech.com/mid-360/specs. Important parameters:

- weight: less than 1kg,
- battery life: up to 5 hours,
- suggested speed during data acquisition: walking speed (4km/h),
- LiDAR type: Livox MID360,
- LiDAR non-repetitive scanning pattern,
- LiDAR range 40m @ 10\% reflectivity, 70 m @ 80\% reflectivity,
- Range Precision (1 $\sigma$): up to 2cm (@ 10m),
- Integrated IMU (Inertial Measurement Unit).

# Possible applications:
- caving
- speleology
- surveying
- culture heritage
- environmental management
- geology
- urban search and rescue
- urban mapping
- ground truth for AGV (Automated Guided Vehicle)
- mobile robot navigation
- precision forestry
- agricultural robotics
- underground mining
- education
- entertainment
- forensics
- critical infrastructure inspection
- space exploration
- protection systems
- digital twin content generation
- automation in construction
- etc...

![largescalemapping1](images/100-2.gif)
City level survey (perspective view).
![largescalemapping2](images/100.gif)
City level survey (top view).

![largescalemapping1](images/a.jpg)
3D data from aerial LiDAR mapping.
![largescalemapping2](images/b.jpg)
Aerial LiDAR fused with ground MANDEYE data (fixed issue with missing elevations).

![largescalemapping1](images/a0.jpg)
Construction site.
![largescalemapping2](images/a6.jpg)
Construction site augmented with MANDEYE 3D data.
![largescalemapping2](images/change.jpg)
Construction progress monitoring, scale blue - smallest changes, red - largest changes.
