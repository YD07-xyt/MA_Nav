# MA_Nav    

## build
```sh
    sudo apt update
    sudo apt install cpufrequtils
    sudo apt install libompl-dev
```

mpc casadi
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

```sh
wget http://fishros.com/install -O fishros && . fishros 
rosdepc install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```sh
colcon build
```
or
```sh
colcon build --cmake-args -DHUMBLE_ROS=humble -DCMAKE_BUILD_TYPE=Release  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --symlink-install
```

ros2 service call /save_tunnel_regions std_srvs/srv/Trigger "{}"