<a href="http://www.automationware.it/">
    <img src="doc/img/logoAW.png" alt="Aw logo" title="AutomationWare" align="right" height="40" />
</a>

Authors: 
- [Mattia Dei Rossi](deirossi@automationware.it) - Automationware
- [Alvin Matarozzo](matarozzo@automationware.it) - Automationware
- [Armando Selvija](selvija@automationware.it) - Automationware

# Aw robotics

This workspace has been developed and tested on `ros2 humble`.

TO DO: ADD GIF EXAMPLES


## Local setup and compile
Clone the repository:
```
https://github.com/Automationware/one4all.git
```
Install necessary dependencies:
```
rosdep initi && rosdep fix-permissions && rosdep update \
    && rosdep install --from-paths ./ -i -y --rosdistro humble \
      --ignore-src
```
Build the project:
```
cd one4all/
colcon build --symlink-install
```
Source the project:
```
. install/setup.bash
```
Run the simulation, including:
- Gazebo
- RViz2
- ROS2 navigation stack
- Move-group
```
ros2 launch neo_simulation2 simulation.launch.py
```
Additionally, rqt may be used for a first interaction with the robot (config file could be also loaded inside the app by GUI):
```
rqt --perspective-file
```


## Docker
Build the docker image:
```
cd one4all/
docker build -t gazebo_ros2_control .
```
Run the demo:
```
sudo docker run -it --rm --name gazebo_ros2_control_demo --net host gazebo_ros2_control ros2 launch neo_simulation2 simulation.launch.py gui:=false
```
Locally:
```
gzclient
```

## Expected result
### Movement's Arm Through Move-Group
TO DO: ADD MOVEMENT WITH MOVE GROUP

### Movement's Base Through Nav2
TO DO: ADD BASE NAV (MULTIPOINTS)