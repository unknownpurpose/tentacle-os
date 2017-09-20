# TentacleOS

This repo contains all the packages necessary to run each element of our T1 robot on ROS.

* Packages for the arm, IMU sensors, vision system, and mind.
* Dockerfiles for RaspberryPi (arm and IMU sensors) and Desktop (vision system)
* Docker-compose, for quickly running ROS commands during dev if ROS isn't set up on local OS.

## Build Instructions

In each context you need to `docker build` this directory, giving the path to the appropriate Dockerfile, e.g. `docker build -t fhtagn:base -f deploy/desktop/Dockerfile .`
