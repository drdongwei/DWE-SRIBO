# sribo

## Introduction
**sribo** is a package designed for efficient UAV state estimation. It supports both onboard real-time estimation and offline estimation using recorded ROS bag data.

## Notes
Follow these steps to use the Sribo package:

1. Download the sribo package into your workspace.
2. Navigate to the `sh` directory:
   ```bash
   cd ./sh
3. Estimation

for estimation onboard

```bash
bash sribo_onboard.sh
```

for estimation with recorded rosbag 

```bash
bash sribo_rosbag.sh
```