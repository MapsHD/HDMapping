# Consideration
Implementation loads all data in memory, there is no out-of-core solution utilized in the project.
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
