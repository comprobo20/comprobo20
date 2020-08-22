---
title: "Useful Resources"
toc_sticky: true
toc_h_max: 3
---
## External Resources to Learn ROS

* [A Gentle Introduction to ROS](https://cse.sc.edu/~jokane/agitr/): we will use a couple of chapters from this book, however, as the book is C++-based its utility will be limited for our purposes.
* [The ROS tutorials](http://wiki.ros.org/ROS/Tutorials) are a great place to learn the basics


## ROS Cheatsheet

### Show all topics currently being published
```bash
$ rostopic list
```

### Show details of a message

```bash
$ rosmsg show rospackagename/msgname
```

Notes: works well with tab completion, add -r for original source

### Show Messages Published on a Topic

```bash
$ rostopic echo topic
```

Notes: can be performed on topics and sub-parts of topics

### Publish Messages on a Topic

```bash
$ rostopic pub --rate=x topic message-type
```

Notes: double tap tab,  then it’ll fill out a stub for you.  Rate, x, specifies how often to publish the message.

### Visualize Sensor Data

```bash
$ rosrun rqt_gui rqt_gui
```

Notes: To make a time series plot go to plugins, visualization, plots. Then choose topic (``scan/ranges[0]`` for example). You can also add more than one topic at a time.

### Rviz

```bash
$ rosrun rviz rviz
```

Notes: can also just type rviz
