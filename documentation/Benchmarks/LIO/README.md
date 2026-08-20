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

# Compatible other SOTA LIO algorithms (benchmark)

Download the dataset from [Bunker DVI Dataset](https://charleshamesse.github.io/bunker-dvi-dataset/)

![Bunkier](/images/sx1.png)
![Benchmark](/images/bunker1.png)
![HDMapping LIO](/images/hdmapping-lio.png)

We provide HDMapping-LIO: easy to run, easy to test LiDAR Inertial Odometry that is as accurate as FAST-LIO, FASTER-LIO and much more precise [[movie]](https://youtu.be/UB-Hx7qgey8).


All following algorithms are generating session compatible with 'multi_view_tls_registration_step_2'.
It is designed for a benchmark.

- https://github.com/MapsHD/benchmark-HDMapping-ground-truth (2026, 'under review', [[movie]](https://youtu.be/8sHyUNC3mZs))
- [1] https://github.com/MapsHD/benchmark-HDMapping_LIO-to-HDMapping (2026, 'under review', [[movie]](https://youtu.be/9AUvPTLUcos))
- [2] https://github.com/MapsHD/benchmark-BIEVR-LIO-to-HDMapping (2026, arXiv, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/bievr-lio.bib), movie ToDo)
- [3] https://github.com/MapsHD/benchmark-SE3-LIO-to-HDMapping (2026, ICRA, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/SE3-LIO.bib), movie ToDo)
- [4] https://github.com/MapsHD/benchmark-EllipseLIO-to-HDMapping (2026, arXiv, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/EllipseLIO.bib), movie ToDo)
- [5] https://github.com/MapsHD/benchmark-D-LIO-to-HDMapping (2026, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/D-LIO.bib), movie ToDo)
- [6] https://github.com/MapsHD/benchmark-FORM-to-HDMapping (2026, ICRA, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/form.bib), [[movie]](https://youtu.be/2c2ySFIncyA))
- [7] https://github.com/MapsHD/benchmark-Super-LIO-to-HDMapping (2026, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/Super-LIO.bib), [[movie]](https://youtu.be/CQmLOdV_mQA))
- [8] https://github.com/MapsHD/benchmark-DALI_SLAM-to-HDMapping (2025, Elsevier ISPRS, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/DALI-SLAM.bib))
- [9] https://github.com/MapsHD/benchmark-lidar_odometry_ros_wrapper-to-HDMapping (2025, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/lidar_odometry_ros_wrapper.bib), [[movie]](https://youtu.be/w233P_MZMWk))
- [10] https://github.com/MapsHD/benchmark-SuperOdometry-to-HDMapping (2021, IROS, 2025, ICRA, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/SuperOdometry.bib), [[movie]](https://youtu.be/2bRunzG43sw))
- [11] https://github.com/MapsHD/benchmark-mola_lidar_odometry-to-HDMapping (2025, IJRR, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/mola-lidar-odometry.bib), [[movie]](https://youtu.be/KcVTTurel44)) 
- [12] https://github.com/MapsHD/benchmark-RESPLE-to-HDMapping (2025, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/RESPLE.bib), [[movie]](https://youtu.be/5PAB4xJmMoo)) (ToDo --> add to automated benchmark)
- [13] https://github.com/MapsHD/benchmark-GenZ-ICP-to-HDMapping (2025, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/GenZ-ICP.bib), [[movie]](https://youtu.be/vgGkucOBVg4))
- [14] https://github.com/MapsHD/benchmark-MM-LINS-to-HDMapping (2025, TIV, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/MM-LINS.bib), movie ToDo)
- [15] https://github.com/MapsHD/benchmark-PIN-SLAM-to-HDMapping (2024, TRO, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/PIN-SLAM.bib), movie ToDo)
- [16] https://github.com/MapsHD/benchmark-SR-LIO-to-HDMapping (2024, IROS, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/sr-lio.bib), movie ToDo)
- [17] https://github.com/MapsHD/benchmark-Voxel-SLAM-to-HDMapping (2024, arXiv, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/Voxel-SLAM.bib), 2nd place ICRA HILTI 2023 SLAM Challenge, 1st place ICCV 2023 SLAM Challenge, movie ToDo)
- [18] https://github.com/MapsHD/benchmark-NV-LIOM-to-HDMapping (2024, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/NV-LIOM.bib), movie ToDo)
- [19] https://github.com/MapsHD/benchmark-c3p-voxelmap-to-HDMapping (2024, arXiv, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/c3p-voxelmap.bib), movie ToDo)
- [20] https://github.com/MapsHD/benchmark-I2EKF-LO-to-HDMapping (2024, arXiv, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/I2EKF-LO.bib), [[movie]](https://youtu.be/B2358Gn62Ho))  
- [21] https://github.com/MapsHD/benchmark-LIO-EKF-to-HDMapping (2024, ICRA, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/LIO-EKF.bib), [[movie]](https://youtu.be/R4Cn1LJ4U_E))
- [22] https://github.com/MapsHD/benchmark-iG-LIO-to-HDMapping (2024, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/iG-LIO.bib), [[movie]](https://youtu.be/KlZf7nHeVmI))
- [23] https://github.com/MapsHD/benchmark-GLIM-to-HDMapping (2024, arXiv, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/GLIM.bib), [[movie]](https://youtu.be/zyZDJECqOG0))
- [24] https://github.com/MapsHD/benchmark-Point-LIO-to-HDMapping (2024, JAIS, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/Point-LIO.bib), [[movie]](https://youtu.be/JlD1hDJHcrs))
- [25] https://github.com/MapsHD/benchmark-LOG-LIO2-to-HDMapping (2024, arXiv, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/LOG-LIO2.bib), movie ToDo) 
- [26] https://github.com/MapsHD/benchmark-DLIO-to-HDMapping (2023, ICRA, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/DLIO.bib), [[movie]](https://youtu.be/xFLqFcoAtk8))
- [27] https://github.com/MapsHD/benchmark-SLICT-to-HDMapping (2023, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/SLICT.bib), [[movie]](https://youtu.be/TUaJN7FJOFU))
- [28] https://github.com/MapsHD/benchmark-KISS-ICP-to-HDMapping (2023, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/KISS-ICP.bib), [[movie]](https://youtu.be/GyB8UuQN0Io))
- [29] https://github.com/MapsHD/benchmark-Faster-LIO-to-HDMapping (2022, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/Faster-LIO.bib), [[movie]](https://youtu.be/bV1jgF_m-Zo))
- [30] https://github.com/MapsHD/benchmark-VoxelMap-to-HDMapping (2022, arXiv, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/VoxelMap.bib), [[movie]](https://youtu.be/oRiuvJRNl-c))
- [31] https://github.com/MapsHD/benchmark-DLO-to-HDMapping (2022, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/DLO.bib), [[movie]](https://youtu.be/-UH81mNLw8Q))
- [32] https://github.com/MapsHD/benchmark-CT-ICP-to-HDMapping (2021, arXiv, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/CT-ICP.bib), [[movie]](https://youtu.be/swEsJHwtE50))
- [33] https://github.com/MapsHD/benchmark-FAST-LIO-to-HDMapping (2020, arXiv, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/FAST-LIO.bib), [[movie]](https://youtu.be/ENlaQTtOXEM))
- [34] https://github.com/MapsHD/benchmark-LOAM-Livox-to-HDMapping (2019, arXiv, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/loam_livox.bib), [[movie]](https://youtu.be/MbKHTmUcI2w))
- [35] https://github.com/MapsHD/benchmark-LeGO-LOAM-to-HDMapping (2018, IROS, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/legoloam2018.bib), [[movie]](https://youtu.be/WpFBXe1zKto))

[[qualitative evaluation benchmark movie]](https://youtu.be/C0CcG9vAokY)
[[quantitative evaluation benchmark movie]](https://youtu.be/PsJaXpWFAis)
[[quantitative benchmark]](https://github.com/MapsHD/benchmark-HDMapping-evaluation-of-odometry-and-SLAM)

Benchmark Orchestration
- https://github.com/MapsHD/benchmark-HDMapping-Orchestration 

Datasets
- https://charleshamesse.github.io/bunker-dvi-dataset/
- https://github.com/Jakubach/kitti_to_ros (Michal W)
- https://github.com/Jakubach/kitti_to_hdmapping (Michal W)
- M2DGR https://github.com/SJTU-ViSYS/M2DGR
- M3DSS https://neufs-ma.github.io/M3DSS/index.html (benchmark, dataset)
- NCLT https://robots.engin.umich.edu/nclt/
- Hilti SLAM Challenge https://hilti-challenge.com/dataset-2023 (Michal P)
- Complex Urban https://sites.google.com/view/complex-urban-dataset
- NTU VIRAL https://ntu-aris.github.io/ntu_viral_dataset/
- HeRCULES
- He-LiPR
- AevaScene
- https://thisparticle.github.io/geode/

Algorithms without code
- https://kafeiyin00.github.io/AEOS/ (2026, ISPRS-Elsevier, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/AEOS.bib), no source code)
- VOX-LIO https://www.mdpi.com/2072-4292/17/13/2214 (2025, MDPI Remote Sensing, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/VOX-LIO.bib), no source code)
- SS-LIO: Robust Tightly Coupled Solid-State LiDAR–Inertial Odometry for Indoor Degraded Environments (2025, MDPI electronics, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/SS-LIO.bib), no source code)
- https://www.sciencedirect.com/science/article/abs/pii/S1566253525002052 (2025, Elsevier Information Fusion, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/STATIC-LIO.bib), no source code)
- https://ieeexplore.ieee.org/abstract/document/11206445 (2025, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/SPS-LIO.bib), no source code)
- https://ieeexplore.ieee.org/abstract/document/11045969 (2025, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/Tightly-Coupled-SLAM-With-Imprecise-Architectural-Plans.bib), no source code)
- https://ieeexplore.ieee.org/document/10954274 (2025, IEEE TITS, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/Universal-LiDAR-Odometry-and-Mapping-With-Dual-Channel-Descriptor.bib), no source code)
- https://github.com/kafeiyin00/HCTO (2024, ISPRS-Elsevier, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/HCTO.bib), no source code)
- Fast and Robust LiDAR-Inertial Odometry by Tightly-Coupled Iterated Kalman Smoother and Robocentric Voxels, RC-Vox (2024, IEEE TITS, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/RC-Vox.bib), no source code)
- https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=11419773 (2026, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/Dynamic-ICP.bib)), https://github.com/JMUWRobotics/Dynamic-ICP

Algorithms with unsolved issues
- https://github.com/MapsHD/benchmark-KISS-SLAM-to-HDMapping (2025, IROS, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/kiss-slam.bib)) (not solved issue https://github.com/kpmrozowski/KISS-SLAM-to-HDMapping/issues/1)
- https://github.com/MapsHD/rko_lio (2025, arXiv, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/RKO-LIO.bib)) (not solved issue: https://github.com/marcinmatecki/rko-lio-to-HDMapping/issues/1)
- https://github.com/MapsHD/benchmark-MAD-ICP-to-HDMapping (2024, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/MAD-ICP.bib), compatibilty issue - only trajectory provided, repository inactive)
- https://github.com/Ji1Xingyu/lio_gvm (2024, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/LIO-GVM.bib)) (not solved issues: https://github.com/Ji1Xingyu/lio_gvm/issues/13, https://github.com/Ji1Xingyu/lio_gvm/issues/12, https://github.com/Ji1Xingyu/lio_gvm/issues/7)
- https://github.com/thisparticle/btsa (2025, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/A-Dynamic-Aware-LIO.bib), dynamic scenes) unsolved issue https://github.com/thisparticle/btsa/issues/4
- https://github.com/BrenYi/Light-LOAM (2024, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/Light-LOAM.bib)) (issue) https://github.com/BrenYi/Light-LOAM/issues/6
- https://github.com/clegenti/2fast2lamaa (2025, arXiv, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/2FAST-2LAMAA.bib)) (issue) https://github.com/clegenti/2fast2lamaa/issues/4
- https://github.com/Livox-SDK/LIO-Livox (2026, Livox proprietary, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/LIO-Livox.bib)) (doesn't support non-repetitive lidar https://github.com/Livox-SDK/LIO-Livox/issues/81)

Not supported algorithm
- https://ieeexplore.ieee.org/document/10900461, https://github.com/kafeiyin00/UA-MPC (2025, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/UA-MPC.bib), we do not support rotary LiDAR)
- https://github.com/YangSiri/OR-LIM (2022, we do not support rotary LiDAR)
- https://github.com/MapsHD/benchmark-LiDAR-IMU-Init-to-HDMapping (2022) (extrinsic calibration)
  
Required PCD format:
- https://github.com/KTH-RPL/dufomap (2024, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/dufomap.bib), dynamic scenes)
- https://github.com/KTH-RPL/DynamicMap_Benchmark (2023, ITSC, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/dufomap.bib), dynamic scenes) 

ToDo Build Success:
- https://github.com/PRBonn/rko_lio (2026)
- https://github.com/NKU-MobFly-Robotics/R-VoxelMap (2026, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/R-VoxelMap.bib)) - Marcin
- https://github.com/HViktorTsoi/PV-LIO (2023, no publication) - Marcin
- https://github.com/SlamCabbage/Optimized-SC-F-LOAM (2022, CVCI, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/SC-F-LOAM.bib)) - Marcin
- https://github.com/wh200720041/floam (2021, IROS, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/F-LOAM.bib)) - Piotr 1
- https://github.com/MapsHD/benchmark-LIO-SAM-to-HDMapping (2020, IROS, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/LIO-SAM.bib), with loop closures) - Piotr 2
- https://github.com/herrmy86/Voxel-SLAM-intensity/tree/main
  
ToDo Build issues:
- https://github.com/xuankuzcr/Global-LVBA (2025, no publication)
- https://github.com/sjtuyinjie/Ground-Fusion2 (2025, IROS, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/Ground-Fusion2.bib))
- https://github.com/ethz-asl/COIN-LIO (2024, ICRA, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/COIN-LIO.bib))
- https://github.com/deepuav/AdaTrajLo (2024, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/Traj-LO.bib))
- https://github.com/StephLin/LIO-SEGMOT (2023, ICRA, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/LIO-SEGMOT.bib))
- https://github.com/XikunLiu-huskit/GLIO (2023, TIV, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/GLIO.bib))
- https://github.com/tiev-tongji/LOG-LIO (2023, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/LOG-LIO.bib))
- https://github.com/chengwei0427/ct-lio (2023, no publication)
- https://github.com/minwoo0611/MA-LIO (2023, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/MA-LIO.bib))
- https://github.com/chengwei0427/hm-lio (2023, no publication)
- https://github.com/RoboFeng/RI-LIO (2023, RA-L, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/RI-LIO.bib))
- https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM (2020, ICRA, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/LINS.bib))
- https://github.com/HKUST-Aerial-Robotics/A-LOAM (2019, no publication)
- https://github.com/PRBonn/semantic_suma/ (2019, IEEE/RSJ, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/SuMa.bib))
- https://github.com/gisbi-kim/SC-LeGO-LOAM (2018, IEEE/RSJ, [[BIB]](https://github.com/MapsHD/HDMapping/blob/main/bib/SC-LeGO-LOAM.bib))
  


ToDo: check if addressed
- https://arxiv.org/pdf/2503.12660 - KISS-SLAM
- https://arxiv.org/pdf/2509.06593 - rko-lio
- https://arxiv.org/pdf/2410.05433 - 2FAST-LAMA
- https://arxiv.org/pdf/2410.08935 - VOXEL-SLAM
- https://arxiv.org/pdf/2204.04932 - SC-F-LOAM
- https://www.mdpi.com/1424-8220/22/2/520 - LEGO-LOAM-SC
- https://arxiv.org/pdf/1907.02233 - LINS-SLAM
- https://github.com/hku-mars/fast-livo
- https://github.com/TixiaoShan/LVI-SAM
- https://github.com/HxCa1/BEV-LIO-LC
- https://github.com/hku-mars/fast-livo2
- https://github.com/dongjae0107/LAPS
- https://github.com/eugeniu1994/MAP_LIO
- https://research.buaa.edu.cn/en/publications/ve-liom-a-versatile-and-efficient-lidar-inertial-odometry-and-map/
- https://www.mdpi.com/2072-4292/15/20/5074
- https://www.mdpi.com/2072-4292/17/15/2656
- https://ieeexplore.ieee.org/document/10237251
- CMLGF-LIO https://isprs-archives.copernicus.org/articles/XLIX-B1-2026/71/2026/
- https://github.com/MIT-SPARK/spark-fast-lio
- https://github.com/HViktorTsoi/PV-LIO
- https://xiaofan4122.github.io/Elevator_LIO_Page/
- https://github.com/lovelyyoshino/FAST_LIO2_Noted
- https://github.com/zlwang7/S-FAST_LIO
- https://github.com/HesaiTechnology-Spatial-Perception/FAST_LIO_Hesai
- https://github.com/rsasaki0109/li_slam_ros2
- https://github.com/APRIL-ZJU/Coco-LIC
- ieeexplore.ieee.org/document/11264316/

