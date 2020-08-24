---
title: "Setup Your Computing Environment"
toc_sticky: true
---

The teaching team will be using ROS Noetic with Ubuntu 20.04, but you should be able to use 18.04 and Melodic (note: the instructions for 18.04 are not posted yet, but are very similar).

> While there are other ways to install ROS on a computer (ROS for Windows, ROS through Docker, ROS through Windows Subsystem for Linux, ROS2), you really, really want to use Ubuntu running via dual boot (not as a virtual machine).  We have found that while these other setups work to varying degrees, there are always little issues that will crop up that will likely get in the way of your learning.  The biggest issue you will see is that most of these other setups will not allow you to run robot simulators with hardware-based graphics acceleration.  Given how much we will be using simulators this semester, you really want the superior performance that comes from hardware acceleration.


## Setting up a Dual Boot

In order to setup your computer for dual boot, you need to create a bootable USB thumb drive with Ubuntu 20.04 on it.  Itzgeek has [a nice walkthrough of how to do this](https://www.itzgeek.com/post/how-to-install-ubuntu-20-04-alongside-with-windows-10-in-dual-boot/) that will allow you to take an existing USB thumb drive and convert it into a bootable installer.

A few quick notes:
* We recommend following the instructions under "Create a bootable USB disk".
* When installing Ubuntu you will likely need to **shrink your Windows patition** to make room for Ubuntu.  The instructions linked above show how you can use the Disk Management utility in Windows to accomplish this.  Unfortunately, sometimes you will not be able to shrink your volume in this way.  If this happens to you, we recommend using the Ubuntu installer to shrink your Windows partition.  If you continue to have issues, send us an e-mail (see below).
* You should probably reserve about 50 GB of space for Ubuntu.
* When installing Ubuntu, you should select the options to **Download updates** and **Install third-party software**.
* If you run into any issues with this, send an e-mail to <a href="mailto:paul.ruvolo@olin.edu">Paul Ruvolo</a> and <a href="mailto:mmalley@olin.edu>">Melinda Malley</a>.


*Once you have a freshly installed copy of Ubuntu 20.04, perform the following steps.*

## Install ROS Noetic

Follow this tutorial (make sure to complete the steps for ``ros-noetic-desktop-full``).

## Setup your Catkin Workspace

For more context on what is going on here, see this ROS tutorial.

```bash
$ source /opt/ros/noetic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make
```

Edit your ``~/.bashrc`` file so that the following line appears last in that file:

```bash
$ source ~/catkin_ws/devel/setup.bash
```

You must perform the rest of the setup in a new terminal window.  To make sure you have done the previous steps correct, type roscd.  You should be in the directory ``~/catkin_ws/devel`` if you did the previous steps correctly. 

## Setup Your CompRobo GitHub Repository

First create a fork of the base CompRobo repository.  If you don't know how to create a fork of a repository, check out the relevant GitHub documentation here.  Next, clone your forked repository inside your catkin workspace.  You will also be performing the step of adding the CompRobo repository as a remote.

```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/your-user-name-here/comprobo20.git
$ cd comprobo20
$ git remote add upstream https://github.com/comprobo20/comprobo20.git
$ cd ~/catkin_ws
$ catkin_make
```

If ``catkin_make`` exits with no error messages, then your laptop is ready to use with the simulated Neato!  You may to close your current terminal as we have found that ``sourcing ~/catkin_ws/devel/setup.bash`` again seems to be required for everything to function properly.
