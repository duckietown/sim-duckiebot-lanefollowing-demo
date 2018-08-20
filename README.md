# duckie-ros

Enables the use of ROS on physical and virtual Duckiebots, by exposing the low-lying ZMQ API used by the Duckietown simulator, [gym-duckietown](https://github.com/duckietown/gym-duckietown) and the physical robot, provided by [duckietown-slimremote](https://github.com/duckietown/duckietown-slimremote/).

## TODO

* Finish the README with complete usage instructions.
* Currently only works when the `SubCameraManager` thread in `duckietown-slimremote/pc/camera.py` is a daemon.

## Installation and Prerequisites

To get started, clone this git repository and enter the project directory:

    git clone https://github.com/duckietown/duckie-ros && cd duckie-ros

## Usage

To launch the lane following task on the ROS stack, run the following command:

    docker-compose -f docker-compose-lf-ros.yml pull && \
    docker-compose -f docker-compose-lf-ros.yml up

## Architecture

TODO: Bhairav

This application translates the [ROS message format](http://wiki.ros.org/msg) into the [ZMQ message format](https://rfc.zeromq.org/spec:13/ZMTP/) bidirectionally for common Duckietown message types...
