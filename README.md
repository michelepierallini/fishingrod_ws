Fishingrod workspace the robot model can be found [here](https://github.com/michelepierallini/example-robot-data/tree/devel).

The actuator module at the base is the SEA EM-ACT presented in [Paper](https://ieeexplore.ieee.org/abstract/document/10529546).

The fishingrod is discretized using a classic pseudo-rigid model with spring and damper.
The $K$ and $D$ matrixes in ```fishingrod.urdf``` has been experimentally estiamted via load tests. We use a plugin for simulating the SEA passive joints.

We have _src/test_experiments_ for testing basic trajectory on the first (and only) active joints.

We have a _src/rlq_quad_controller_ for implementing PPO.
Training has been done by implementing RL-baseline PPO via the NVIDIA Omniverse framework [fishing_rod_real_pos.py](https://github.com/michelepierallini/OmniIsaacGymEnvs/blob/main/omniisaacgymenvs/tasks/fishing_rod_real_pos.py).

The interference Node is mainly adapted from [loco_sim](https://github.com/CentroEPiaggio/locosim_ws).

# Usage in Simulations Interference and Classic Trajectory
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
--------------------------------------------------------------------------------------------------------------

If you want to use a "test" trajectory, use the package `test_experiments` and run 
(i) as before for loading the controllers 

(ii) 
```
source /opt/ros/humble/setup.bash
. install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch test_experiments test_sim.launch.py
```

Model based controller DDP from [aslr_to](https://github.com/michelepierallini/aslr_to) this [branch](https://github.com/michelepierallini/aslr_to/tree/my-fishing-rod-update).

```
source /opt/ros/humble/setup.bash
colcon build --symlink-install && . install/setup.bash
export ROS_DOMAIN_ID=10
ros2 launch ddp_fishing ddp_sim.launch.py 
```

# Usage of the SEA plugin

Compile the ```sea_plugin```

```
cd fishingrod_ws/src/sea_plugin_package/
mkdir build && cd build
cmake .. && make -j4 && sudo make install 
```

if the SEA plugin returns error please run (```. install/setup.bash```) 
``` 
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:~/install/serial_elastic_plugin/lib

```

# Dockerfile 

Please refer to this [link](https://github.com/ddebenedittis/solo12_exp/tree/main)

Briefly setup the [Dokcer](https://docs.docker.com/engine/install/ubuntu/) and [NVIDIA Container Tollkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). Then, run
```
./build.bash [-r]
``` 
```
 ./run.bash
``` 
and inside the container  run
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && source install/setup.bash
```

# Note

*INFO*: the system is very sensitive to damping terms due to noise in the velocity so reduce the damping term in the case of violent oscillations.

*INFO*: change this path in ```fishingrod.urdf```, i.e., (also in the Docker container)
```
<parameters>/home/michele/michele_try_ws/fishingrod_ws/src/fishingrod_gazebo/config/fishingrod_gazebo_sim_jnt_PD.yaml</parameters> 
```

*INFO*: the package `throw_experiment` is yet to be completed.

# TODO

1) Implement the camera sensor 
2) `throw_experiment` package. If I will try experiments. Not planned yet
3) Creates the reel module
4) Add IMU to control loop in the DDP-framework
-----------------------------------------------------------------------------------------------
-) *Doing* Retrain the Network with smart observation and/or implementing a curriculum learning. 


