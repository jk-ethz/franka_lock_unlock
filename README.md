# Franka Lock Unlock

Locking and unlocking of Franka Emika Panda joint brakes programmatically. Open or close all joint locks either from the command-line, or from any Python program.

Also supports the activation of the Franka Control Interface (FCI) and other options.

## Problem Description

While the Franka Panda is a great robot for research, it lacks the option of unlocking or locking its joints from a different source other than the Franka Desk Web UI. That is now possible with the introduction of this package.

## Command-Line Usage

```
usage: ./__init__.py [-h] [-u] [-l] [-w] [-r] [-p] [-c] hostname username password

positional arguments:
  hostname          The Franka Desk IP address or hostname, for example "1.2.3.4".
  username          The Franka Desk username, usually "admin".
  password          The Franka Desk password.

optional arguments:
  -h, --help        show this help message and exit
  -u, --unlock      Unlock the brakes.
  -l, --relock      Relock the brakes on exit.
  -w, --wait        Wait in case the robot web UI is currently in use.
  -r, --request     Request control by confirming physical access to the robot in case the robot web UI is currently in use.
  -p, --persistent  Keep the connection to the robot open persistently.
  -c, --fci         Activate the FCI.
```

## ROS Package

This repository exports a ROS package. It can be installed and invoked by the following commands.

### Installation

```
catkin build franka_lock_unlock
```

### Usage
```
rosrun franka_lock_unlock __init__.py <PARAMS>
```
