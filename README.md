# MA_Nav    

## env


```sh
    sudo apt update
    sudo apt install cpufrequtils
    sudo apt install libompl-dev
```

mpc  
osqp osqpEigen

qpOASES
```bash
git clone https://github.com/coin-or/qpOASES.git
cd ./qpOASES
mkdir build && cd ./build
cmake ..
make 
sudo make install
```

OsqpEigen

```bash
git clone https://github.com/gbionics/osqp-eigen.git
cd ./osqp-eigen
mkdir build && cd ./build
cmake ..
make 
sudo make install
```


casadi(不必须)
```sh
    sudo apt-get install swig
    git clone https://github.com/casadi/casadi.git
    cd casadi
    mkdir build
    cd build
    cmake -DWITH_PYTHON=ON -DWITH_PYTHON3=ON -DWITH_IPOPT=ON _DWITH_MUMPS=ON ..
    make
    sudo make install
```

ros2
```sh
wget http://fishros.com/install -O fishros && . fishros 
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```


## build
```sh
colcon build
```
or
```sh
colcon build --cmake-args -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install
```


## run

```sh
source install/setup.sh
ros2 launch ma_nav run.launch.py
```
建图or导航
planner.yaml中的mapping_model 更改


保存服务的话题在planner.yaml中  save_map_srv_topic save_tunnel_regions_srv_topic'

建图保存地图
```sh
ros2 service call /save_global_map std_srvs/srv/Trigger "{}"
```

保存地图隧道语义
```sh
ros2 service call /save_tunnel_regions std_srvs/srv/Trigger "{}"
```

可视化
```sh
rviz2 -d ./nav.rviz
```