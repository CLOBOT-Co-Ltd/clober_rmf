# clober_rmf

## 1. RMF
[`Robot Middleware Framework(RMF)`](https://osrf.github.io/ros2multirobotbook/intro.html#so-what-is-rmf) is a collection of reusable, scalable libraries and tools building on top of ROS 2 that enable the interoperalibilty of heterogeneous fleets of any type of robotic systems. You can read the `RMF`'sdocuments [here](https://osrf.github.io/ros2multirobotbook/intro.html), and you can reference the original source code in here, [osrf/rmf_core](https://github.com/osrf/rmf_core).

## 2. Install RMF Packges

Before install the clober_rmf packges, you must [install the RMF packges](https://osrf.github.io/ros2multirobotbook/intro.html#setup-sources-and-installation-of-rmf).

```bash
curl -s http://rmf.servehttp.com/repos.key | sudo apt-key add -
sudo sh -c 'echo "deb http://rmf.servehttp.com/ubuntu/main/ `lsb_release -cs` main" > /etc/apt/sources.list.d/rmf.list'
sudo apt-get update
sudo apt-get install ros-foxy-ament-cmake-catch2 ros-foxy-building-gazebo-plugins ros-foxy-building-map-msgs ros-foxy-building-map-tools ros-foxy-rmf-cmake-uncrustify ros-foxy-rmf-dispenser-msgs ros-foxy-rmf-door-msgs ros-foxy-rmf-fleet-adapter ros-foxy-rmf-fleet-msgs ros-foxy-rmf-lift-msgs ros-foxy-rmf-task-msgs ros-foxy-rmf-traffic-msgs ros-foxy-rmf-traffic-ros2 ros-foxy-rmf-traffic ros-foxy-rmf-utils ros-foxy-traffic-editor
```

## 3. Install Clober RMF

Install the clober RMF in foxy
```bash
mkdir -p ~/rmf_ws/src
cd ~/rmf_ws/src
git clone -b foxy-devel https://github.com/CLOBOT-Co-Ltd/clober_rmf.git
```

Source ROS2 and build,
```bash
cd ~/rmf_ws
source /opt/ros/foxy/setup.bash
colcon build
```

## 4. Install Clober Free Fleet
For using the Clober RMF packages, you must install the [Clober Free Fleet packges](https://github.com/CLOBOT-Co-Ltd/clober_free_fleet) and understand the structure of free_fleet.

## 5. Examples

To simulate the Clober RMF, you must use `clober_rmf` packges with `clober_free_fleet` packages

### 5.1 Single Clober RMF

#### 5.1.1 Launch clober free fleet client

Launch the clober free fleet client in ROS 1 terminal (noetic):

```bash
source /opt/ros/noetic/setup.bash
source ~/client_ws/install/setup.bash
roslaunch clober_ff_client_ros1 clober_3x3_ff.launch
```

#### 5.1.2 Launch clober free fleet server

Launch the clober free fleet server in ROS 2 terminal (foxy):

```bash
source /opt/ros/foxy/setup.bash
source ~/server_ws/install/setup.bash
ros2 launch clober_ff_server_ros2 clober_world_ff.xml
```

#### 5.1.3 Launch clober RMF

Launch the clober rmf in ROS 2 terminal (foxy):

```bash
source /opt/ros/foxy/setup.bash
source ~/rmf_ws/install/setup.bash
ros2 launch clober_rmf clober_rmf.launch.xml
```

### 5.2 Multi Clober RMF

#### 5.2.1 Launch clober free fleet client

Launch the clober free fleet client in ROS 1 terminal (noetic):

```bash
source /opt/ros/noetic/setup.bash
source ~/client_ws/install/setup.bash
roslaunch clober_ff_client_ros1 multi_clober_3x3_ff.launch
```

#### 5.2.2 Launch clober free fleet server

Launch the clober free fleet server in ROS 2 terminal (foxy):

```bash
source /opt/ros/foxy/setup.bash
source ~/server_ws/install/setup.bash
ros2 launch clober_ff_server_ros2 clober_world_ff.xml
```

#### 5.2.3 Launch clober RMF

Launch the clober rmf in ROS 2 terminal (foxy):

```bash
source /opt/ros/foxy/setup.bash
source ~/rmf_ws/install/setup.bash
ros2 launch clober_rmf clober_rmf.launch.xml
```

## 6. Traffic Editor

If you want to simulate the clober in another maps, you should make the `map images`([by using slam](https://github.com/CLOBOT-Co-Ltd/clober)), `.building.yaml` files and `nav_graph.yaml` files.
You can download the [`rmf_traffic_editor`](https://github.com/open-rmf/rmf_traffic_editor/tree/main/rmf_traffic_editor).

* The method of making the `.building.yaml` files: https://osrf.github.io/ros2multirobotbook/traffic-editor.html
* The method of making the `nav_graph.yaml` files: https://osrf.github.io/ros2multirobotbook/simulation.html#building-map-generator

