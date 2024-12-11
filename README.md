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

# Usage of the SEA plugin

Infos: the system is very sensitive to damping terms due to noise in the velocity so reduce the damping term in the case of violent oscillations

Compile the ```sea_plugin```

```
cd fishingrod_ws/src/sea_plugin_package/
mkdir build && cd build
cmake .. && make -j4 && sudo make install 
```

if the SEA plugin returns error please run 
``` 
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/michele_try_ws/fishingrod_ws/install/serial_elastic_plugin/lib

```

# TODO

1) Test the IMU
2) Implement the camera (I do not think IMU will solve)
3) Callbacks for plots
-------------------------------------------------------------
-) Retrain the Network with smart observation


