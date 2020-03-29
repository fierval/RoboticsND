# Go Chase it!

![RNProject2](https://user-images.githubusercontent.com/39171152/77837222-89500580-711b-11ea-99c1-c47fd2a75b39.gif)

## Setup

* Clone the repo
* From the root of the cloned repo:

```sh
$ cd Project2/catkin_ws
$ catkin_make
$ source devel/setup.bash
```

* Optionally add a call to `source devel/setup.bash` to `.bashrc` from the currently built workspace `devel directory`. Otherwise issue `source devel/setup.bash` in each new terminal window.

* From the current shell prompt:
```sh
$ roslaunch my_robot world.launch
```

* From the new terminal window:

```sh
$ cd Project2/catkin_ws
$ source devel/setup.bash
$ roslaunch ball_chaser ball_chaser.launch
```



