# duckie-ros

[![Docker Hub](https://img.shields.io/docker/pulls/duckietown/duckie-ros.svg)](https://hub.docker.com/r/duckietown/duckie-ros)

Enables the use of ROS on physical and virtual Duckiebots, by exposing the low-lying ZMQ API used by the [Duckietown simulator](https://github.com/duckietown/gym-duckietown) and the [physical robot stack](https://github.com/duckietown/rpi-ros-kinetic-base). It is based on [ROS Kinetic Kame](http://wiki.ros.org/kinetic) and has some useful [communication](https://github.com/duckietown/duckietown-slimremote/) and vision libaries.

## TODO

* Finish the README with complete usage instructions.
* Currently only works when the `SubCameraManager` thread in `duckietown-slimremote/pc/camera.py` is a daemon.

## Installation and Prerequisites

To get started, fork or clone this git repository and enter the project directory:

    git clone https://github.com/duckietown/duckie-ros && cd duckie-ros

## Usage

To launch the lane following task, run the following command:

    docker-compose -f docker-compose-lf-ros.yml pull && \
    docker-compose -f docker-compose-lf-ros.yml up

This will give you a reward output like:

    duckie-ros_1   | starting sub socket on 8902
    duckie-ros_1   | listening for camera images
    duckie-ros_1   | [Challenge: LF] The average reward of 10 episodes was -315.7005. Best episode: -283.9803, worst episode: -368.1122
    duckie-ros_duckie-ros_1 exited with code 0

You can terminate the run at any time by pressing <kbd>CTRL</kbd>+<kbd>c</kbd>.

## Write your own agent

To write your own ROS agent, first fork this repository, and edit the file [`agent-ros.py`](agent-ros.py). Then run the following command from the root directory of this project on your local machine to evaluate its performance:

    docker build -t duckietown/duckie-ros . && \
    docker-compose -f docker-compose-lf-ros.yml up`

Then check the average reward and try to improve your score. Good luck!

## Architecture

TODO: Bhairav

This application translates between [ROS messages](http://wiki.ros.org/msg) and [ZMQ messages](https://rfc.zeromq.org/spec:13/ZMTP/) bidirectionally for common Duckietown message types...
