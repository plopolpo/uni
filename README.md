
# <TITLE>

<TITLE> is a small collection of ROS2 nodes which interact in order to send and collect rgb and depth frames from the OAKD camera. 


## Nodes

Those are the nodes you can run with ROS2 

| Node name   | Description                                                   |
| :--------   | :----------------------------------------------------------   |
| `applicant` | It sends each `TIMER_PERIOD` a message to `ACTION_TOPIC_NAME` |
| `publisher` | When it receives a message on `ACTION_TOPIC_NAME` it sends both the rgb and depth frames to `RGB_TOPIC_NAME` and `DEPTH_TOPIC_NAME` respectively|
| `subscriber`| Displays the rgb and depth images received from `RGB_TOPIC_NAME` and `DEPTH_TOPIC_NAME` in an OpenCV window |



## Setup Locally

- [Setup ROS2 humble](https://docs.ros.org/en/humble/Installation.html)

- [Setup Depthai](https://docs.luxonis.com/software/depthai/manual-install/)

- Clone the project

```bash
  $ git clone https://github.com/plopolpo/uni.git
```

- Go to the project directory

```bash
  $ cd uni
```

- Source the environment for the current ROS2 installation

```bash
  $ source /path/to/ros2-humble/setup.bash
```

- Install the missing dependencies

```bash
  $ rosdep install -i --from-path src --rosdistro humble -y
```

- Compile the project

```bash
  $ colcon build
```
 

## Run Locally

- Plug in [OAKD](https://shop.luxonis.com/products/oak-d) camera

- Go to the project directory

```bash
  $ cd uni
```

- Source the environment for the current ROS2 installation and project

```bash
  $ source /path/to/ros2-humble/setup.bash
  $ source install/local_setup.bash
```

- Run the node

```bash
  $ ros2 run luxonis <node-name>
```
 


## Authors

- [Polo Leonardo](https://www.github.com/PloGaming)
- Nolli Nathhan
- Kumar Neeraj
