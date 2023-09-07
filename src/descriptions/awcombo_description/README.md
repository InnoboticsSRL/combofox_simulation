# Awcombo_description
This package contain the AwCombo as whole robot, connecting all the sub-elements:
- awcombo base &emsp;&emsp; -> &emsp; in package: amr_combo_v2
- awtube2_18kg arm &ensp; -> &emsp; in package: awtube2_18kg_v2
- gripper &emsp;&emsp;&emsp;&emsp;&emsp;&emsp;  -> &emsp; in package: grip_exon
- camera support &emsp;&emsp; -> &emsp; in package: camera_support
- realsense camera &emsp;&nbsp; -> &emsp; in package: realsense2_description

## Display
Showing the AwCombo in Rviz (after building and sourcing the package):
``` 
ros2 launch awcombo_description display.launch.py
```
