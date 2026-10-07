# Consideration
Most of the implementation keeps its data in memory. Lidar odometry (step 1) has two parameters that bound its memory
use on long recordings (below); for the rest, virtual memory on a fast SSD lets you work with large projects.

# Lidar odometry on long recordings
Two parameters in the `[performance]` section of the lidar odometry parameter file (`LidarOdometryParams`) keep the
memory of `lidar_odometry_step_1` bounded. Neither changes the results: the trajectory, scans and poses are the same
with or without them.

| Parameter | Default | What it does |
|---|---|---|
| `lazy_load_raw_clouds` | `true` | Each raw point-cloud file is loaded when step 1 reaches its time range and freed once step 1 has passed it, instead of loading every file before step 1 starts. |
| `points_global_spill_directory` | `""` (in memory) | When set, the map buffer of step 2 (every frame's points between two sliding-window resets) is kept in a temporary file in this directory instead of in memory. The file is removed when step 2 ends. |

For a long recording from a slow-moving platform (many frames inside one sliding window), set
`points_global_spill_directory` to a local SSD with room for the buffer (tens of GB on a recording of about an hour):

```toml
[performance]
lazy_load_raw_clouds = true
points_global_spill_directory = "/path/to/fast/scratch"
```

Notes:
- With `lazy_load_raw_clouds`, every raw file is read twice: once when the data is loaded (to record its time range)
  and again during step 1. The input files must not change during the run; if one does, step 1 stops with an error.
- The spill directory must exist. During a sliding-window reset the old buffer and the retained half exist side by
  side, so allow about 1.5 times the buffer size. A write error ends step 2 with an error, and the spill files are
  removed.
- Results are identical to running without these parameters, as long as the per-frame time limit
  (`real_time_threshold_seconds`) is not reached: when it is, the number of iterations depends on machine speed, with
  or without them.

With modern operating systems, you can work with large projects, utilizing virtual memory with large and fast SSD.

The setup of optimal paging is system-dependent.

# Windows 11

There is no extra step to take, make sure that you have plenty of free space on the system drive.
The amount of data of 30x - 50x of dataset size is recommended.

# Windows 10
- Prepare enough free space on the hard drive, the amount of data of 50x of dataset is recommended.
- Go to the "Advanced system settings", you can do it, by running `sysdm.cpl` as admin.
- Click the `Advanced` tab.
- Click the `Settings` button inside the `Performance` section.
- Click the `Advanced` tab.
- Click the `Change` button inside the `Virtual memory` section.
- Turn off the `Automatically manage paging files size for all drives` option.
- Select the Custom size option.
- Select the fastest drive (fast SSD if possible) in `Paging file size for each drive`
- Specify the initial and maximum size for the paging file in megabytes, Initial 30x dataset size, maximum, 50x dataset size.
- Click the Set button.
- Click the OK button and OK button again.
- Restart your computer

# Linux (Ubuntu 22.04)
In Linux, there are multiple ways to create swap space. Let us introduce a simple, but not persistent one.
Create a swap file, adjust `of=/swapfile` location and ` count=32` accordingly.
```
sudo dd if=/dev/zero of=/swapfile bs=1G count=32
``` 
Adjust permission of created file:
```
sudo chmod 600 /swapfile
```

Use file as swap:
```
sudo mkswap /swapfile
sudo swapon /swapfile
```

Verify if a swap is available:
```
sudo swapon --show
```
