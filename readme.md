Fishingrod workspace to implement PPO. 
[Paper](https://ieeexplore.ieee.org/abstract/document/10529546)

Training has been done by implementing RL-baseline PPO via the NVIDIA Omniverse framework 

# Usage in Simulations
(i)
```
source /opt/ros/humble/setup.bash
colcon build --symlink-install && . install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch fishingrod_gazebo sim.launch.py
 ```
(ii)
```
source /opt/ros/humble/setup.bash
colcon build --symlink-install && . install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch rlg_quad_controller fishingrod_simulation.launch.py
```

# TODO
1) Add stiffnees (real) to the ```.urdf```
2) Test the IMU
3) Implement the camera (I do not think IMU will solve)
4) Plug-in for stiffness and damping (eventually)