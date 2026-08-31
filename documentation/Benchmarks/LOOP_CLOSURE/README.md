# Automatic loop closure

We provide easy tool that will help You annotating ground truth SLAM loop closures. You mark pair of point clouds that forms edge, then use ICP like pair wise registration to generatie ground truth for AI training.
Annotated ground truth edges for LiDAR pose graph slam are available at [ground truth edges](https://zenodo.org/records/18527593).
Movie how to use it is available at [prepare training dataset movie](https://youtu.be/LOGpVg1Nc3k).

[![GROUND-TRUTH-EDGE-ANNOTATION-TOOL](https://img.youtube.com/vi/tJdNNfBubeM/default.jpg)](https://youtu.be/tJdNNfBubeM)
[![MANUAL-LOOP_CLOSURE-TOOL](https://img.youtube.com/vi/6SNvrZ2ROKg/default.jpg)](https://youtu.be/6SNvrZ2ROKg)

## AI loop closure benchmark [https://github.com/MapsHD/benchmark-HDMapping-AILoopClosure-Orchestration](https://github.com/MapsHD/benchmark-HDMapping-AILoopClosure-Orchestration)
- [1] https://github.com/MapsHD/benchmark-HDMapping-AILoopClosure-LCR-Net (in progress)
- [2] https://www.open3d.org/docs/latest/python_api/open3d.registration.compute_fpfh_feature.html
- [3] https://github.com/HxCa1/BEV-LIO-LC/blob/main/README.md
- [4] https://huggingface.co/magic-leap-community/superglue_indoor + BEV
- [5] https://link.springer.com/article/10.1007/s41315-025-00516-5
- [6] https://ieeexplore.ieee.org/abstract/document/10251438
- [7] https://github.com/yutongwangBIT/SP-Loop
- [8] https://github.com/caodanyang/FUSIONLCD
- [9] https://www.mdpi.com/2504-446X/8/7/322
- [10] https://github.com/MIT-SPARK/KISS-Matcher

## LiDAR loop closure benchmark [https://github.com/MapsHD/benchmark-HDMapping-LiDAR-LoopClosure-Orchestration](https://github.com/MapsHD/benchmark-HDMapping-LiDAR-LoopClosure-Orchestration)
- [1] DALI-SLAM
- [2] MM-LINS
- [3] PIN-SLAM
- [4] voxel-slam
- [5] nv-liom
- [6] glim 
- [7] slict
- [8] lego-loam

## Interesting links
- BTC: A Binary and Triangle Combined Descriptor for 3D Place Recognition Unlisted [[YT]](https://www.youtube.com/watch?v=zB4Xqi3-J9U)
- https://github.com/MapsHD/SemanticLoopClosure
- https://github.com/mit-spark/kiss-matcher
- https://github.com/zjuluolun/BEVPlace2
- https://github.com/csiro-robotics/LoGG3D-Net
- https://github.com/PRBonn/OverlapNet
- https://arxiv.org/pdf/2607.21138
