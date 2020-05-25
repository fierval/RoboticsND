# Home Service Robot

The robot drives around a house picks up and drops off objects. We are using [turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/) for this task

## Video

[![Turtlebot3 Navigation](https://img.youtube.com/vi/dUE83tnyloI/0.jpg)](https://www.youtube.com/watch?v=dUE83tnyloI?t=5)


## Prerequisits
1. Ubuntu 18.04
1. ROS melodic
1. Gazebo 9.12
1. GCC 7.5

## Installation

1. Install turtlebot3:

    ```sh
    sudo apt update
    sudo apt install ros-melodic-turtlebot3*
    ```
2. Set turtlebot3 model
    ```sh
    echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
    ```
3. Clone this repo
    ```sh
    mkdir ~/git && cd git
    git clone https://github.com/fierval/RoboticsND.git
    ```
4. Build everything
    ```sh
    cd RoboticsND/Project7/catkin_ws
    catkin_make
    ```
5. Add the follwoing to `~/.bashrc`
    ```sh
    PROJECT=Project7
    source /opt/ros/melodic/setup.bash
    cd ~/git/udacity/robotics/RoboticsND/$PROJECT/catkin_ws
    source devel/setup.bash
    ```
6. `source ~/.bashrc` or restart terminal
