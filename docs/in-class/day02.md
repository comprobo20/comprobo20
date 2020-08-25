---
title: "Coordinate Frames and Basic ROS Concepts"
toc_sticky: true
---

## Today
* Coordinate frames and coordinate transformations
* Questions / clarifications on basic ROS concepts
* Writing our first ROS node
* (if time) Work on the Warmup Project

## For Next Time
* Find a partner for the Warmup Project and get started (there is an intermediate deliverable due on class 4).

## Coordinate Frames and Coordinate Transforms in Robotics

> Likely you've encountered the notion of multiple coordinate systems before at some point in your academic career.  Depending on your path through Olin, you may already be familiar with the mechanics of how to map vectors between different coordinate systems (either in 2D or 3D).  In this exercise, you'll get a chance to refresh some of this knowledge and to also gain a conceptual understanding of how the notion of multiple coordinate systems plays out in robotics.

Suppose your Neato is at position 3.0m, 5.0m with a heading of 30 degrees (where counter-clockwise rotation is positive) in a coordinate system called ``world``.  Draw picture.  Make sure to label the axes of the world coordinate system (don't worry about the z-axis).

In robotics, we frequently need to express the position of various entities (e.g., obstacles, goal locations, other robots, walls, doorways, etc.).  While we could express all of these positions in terms of the coordinate system world, in many situations this will be cumbersome.

**Exercise:** Taking the Neato as an example, make a list of the coordinate systems that you feel would be convenient to define.  For each coordinate system, define its origin and give a few examples of entities that would be natural to express in the coordinate system. TODO: more scaffolding

### base_link

Next, we'll define ``base_link``, which will serve as our robot-centric coordinate system.  The origin of this coordinate system will be at the midpoint of the line connecting the robot's wheels.  The x-axis will point forwards, the y-axis will point to the left, and the z-axis will point up.  Update your drawing to indicate the position of the base_link coordinate axes (again, don't worry about the z-axis).

Now that we have defined our new coordinate system, we'd like to be able to take points expressed in this coordinate system and map them to the world coordinate system (and vice-versa).  In order to do this, we need to specify the relationship between these two coordinate systems.  A natural way to specify the relationship between two coordinate systems is to specify the position of the origin of one coordinate system in the other as well as the directions of the coordinate axes of one frame in the other.  Going back to our original example we can say that the coordinate axes of the Neato's base_link coordinate system are at position 3.0m, 5.0m with a rotation of 30 degrees relative to the coordinate axes of the world coordinate frame.  We usually think of this information as defining the transformation from world to base_link.  It turns out that with just this information, we can map vectors between these two coordinate systems.  ROS has robust infrastructure to handle these transformations automatically, so for the most part when writing ROS code, you don't have to worry about how to actually perform these transformations.  However, to build your understanding, we'll dig into this the details a bit.

### From ``base_link to world``

**Exercise:** Determine the coordinates of a point located at (1.0m, 0.0m) in the base_link coordinate system in the world coordinate system.  First draw the point on the board to make sure everyone agrees what its location is.  Once you've determined your answer, how can you tell if you are right?

**Exercise:** Determine the coordinates of a point located at (0.0m, 1.0m) in the base_link coordinate system in the world coordinate system.  First draw the point on the board to make sure everyone agrees what its location is.  Once you've determined your answer, how can you tell if you are right?


**Exercise:**  Determine the coordinates of a point located at (x, y) in the base_link coordinate system in the world coordinate system.  If you are having trouble operationalizing your answer in terms of equations, you can define it in terms of high-level operations (e.g., translations, rotations, etc.).


### From ``world`` to ``base_link``

There are multiple ways to tackle this one.  I think it's easiest to do algebraically, but you can do it in terms of geometry / trigonometry too.  Don't get too hung up on the mechanics, try to understand conceptually how you would solve the problem.

**Exercise:** Determine the coordinates of a point located at (0.0m, 1.0m) in the world coordinate system in the base_link coordinate system.  First draw the point on the board to make sure everyone agrees what its location is.  Once you've determined your answer, how can you tell if you are right?


**Exercise:** Determine the coordinates of a point located at (1.0m, 0.0m) in the world coordinate system in the base_link coordinate system.  First draw the point on the board to make sure everyone agrees what its location is.  Once you've determined your answer, how can you tell if you are right?

**Exercise:**  Determine the coordinates of a point located at (x, y) in the world coordinate system in the base_link coordinate system.  If you are having trouble operationalizing your answer in terms of equations, you can define it in terms of high-level operations (e.g., translations, rotations, etc.).


### Static Versus Dynamic Coordinate Transformations

The relationship between some coordinate systems are dynamic (meaning they change over time) and some are static (meaning they are constant over time).

**Exercise:**  Assume that our Neato robot can move about in the world by using its wheels.  Is the relationship between world and base_link static or dynamic?  Given the coordinate systems you came up with earlier, list some examples of coordinate system relationships that are static and some that are dynamic.
Before Starting


## Coding Exercises

> Note: Use the following link to find [sample Solutions for these coding exercises](../Sample_code/day02_solutions).  You can also find this code in your `comprobo20` repository.  If you don't see it, try ``$ git pull upstream``.

### Creating a ROS package

Let's write our code today in a package called in_class_day02 (Note: to avoid merge conflicts, I'll be checking in a sample solution under in_class_day02_solution).  To create the package run the following commands:

```bash
$ cd ~/catkin_ws/src/comprobo18
$ catkin_create_pkg in_class_day02 rospy std_msgs geometry_msgs sensor_msgs
```

### Creating ROS Messages in a Python Program (walkthrough in main room)

ROS messages are represented in Python as objects.  In order to create a ROS message you must call the ``__init__`` method for the ROS message class.  As an example, suppose we want to create a ROS message of type ``geometry_msgs/PointStamped``.  The first thing we need to do is import the Python module that defines the ``PointStamped`` class.  The message type ``geometry_msgs/PointStamped`` indicates that the ``PointStamped`` message type is part of the ``geometry_msgs`` package.  All of the definitions for messages stored in the ``geometry_msgs`` package will be in a sub-package called ``geometry_msgs.msg``.  In order to import the correct class definition into our Python code, we can create a new Python script at ``~/catkin_ws/src/in_class_day02/scripts/test_message.py`` and add the following line to our Python script.

```python
#!/usr/bin/env python3
""" This script explores publishing ROS messages in ROS using Python """
from geometry_msgs.msg import PointStamped
```

> Note: the first line tells the shell how to execute your script.

Now we will want to create a message of type PointStamped.  In order to do this, we must determine what attributes the PointStamped object contains.  In order to do this, run

```bash
$ rosmsg show geometry_msgs/PointStamped
Which will generate the output:
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point point
  float64 x
  float64 y
  float64 z
```

If we look at the lines that are unindented (aligned all the way to the left), we will see the attributes that comprise a ``PointStamped`` object.  These attributes are header (which is of type ``std_msgs/Header``) and point (which is of type ``geometry_msgs/Point``).  The indented lines define the definition of the ``std_msgs/Header`` and ``geometry_msgs/Point`` messages.  To see this, try doing running ``$ rosmsg show`` for both ``std_msgs/Header`` and ``geometry_msgs/Point``.

> **Cool trick:** if you run ``$ rosmsg show -r geometry_msgs/PointStamped`` you will see any comments that were included in the original ROS message file (this can help to understand what each field means).

In order to create the PointStamped object, we will have to specify both a ``std_msgs/Header`` and a ``geometry_msgs/Point``.  Based on the definitions of these two types given by ``$ rosmsg show`` (output omitted, but you can see it in a slightly different form above), we know that for the ``std_msgs/Header`` message we need to specify seq, stamp, and frame_id. It will turn out that we don't have to worry about the ``seq`` (it will automatically be filled out by the ROS runtime when we publish our message), the stamp field is a ROS time object (see this tutorial), and the ``frame_id`` field is simply the name of the coordinate frame (more on coordinate frames later) in which the point is defined.  Likewise, the ``geometry_msgs/Point`` object needs three floating point values representing the $$x$$, $$y$$, and $$z$$ coordinates of a point.  We can create these two messages using the standard method of creating objects in Python.  In this example we will be using the keyword arguments form of calling a Python function which will make your code a bit more robust and a lot more readable.  First, we add the relevant import statements:

```python
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import rospy
```

Now we can define the header and point that will eventually makeup our PointStamped message.

```python
rospy.init_node('test_message')    # initialize ourselves with roscore
my_header = Header(stamp=rospy.Time.now(), frame_id="odom")
my_point = Point(1.0, 2.0, 0.0)
```

Now that we have the two fields required for our PointStamped message, we can go ahead and create it.

```python
my_point_stamped = PointStamped(header=my_header, point=my_point)
```

To see what our resultant message looks like, we can print it out:

```python
print(my_point_stamped)
```

This will produce the output:

```bash
header: 
  seq: 0
  stamp: 
    secs: 1441500244
    nsecs: 244467020
  frame_id: odom
point: 
  x: 1.0
  y: 2.0
  z: 0.0
```

> Note that instead of creating the two attributes of PointStamped in separate lines, we can do everything in one line as:
```python
my_point_stamped = PointStamped(header=Header(stamp=rospy.Time.now(),
                                              frame_id="odom"),
                                point=Point(1.0, 2.0, 0.0))
```

In order to do something interesting, let's publish our message to a topic called ``/my_point``

```python
publisher = rospy.Publisher('/my_point', PointStamped, queue_size=10)
# rospy.Rate specifies the rate of the loop (in this case 2 Hz)
r = rospy.Rate(2)
while not rospy.is_shutdown():
    my_point_stamped.header.stamp = rospy.Time.now()    # update timestamp
    publisher.publish(my_point_stamped)
    r.sleep()
```
Try running your code!  Before, you run your code using rosrun, you need to make your code executable:

```bash
$ chmod u+x ~/catkin_ws/src/in_class_day02/scripts/test_message.py
```

Run your code:
```bash
$ rosrun in_class_day02 test_message.py
```

How can you be sure whether it is working or not?  Try visualizing the results in rviz.  What steps are needed to make this work?


### Callbacks (walkthrough in main room)

[Callback functions](https://en.wikipedia.org/wiki/Callback_(computer_programming)) are a fundamental concept in ROS.  Specifically, they are used to process incoming messages inside a ROS node once we have subscribed to a particular topic.  Let's write some code to listen to the message we created in the previous step.

First, let's create a new ROS node in a file called ``receive_message.py`` in the directory ``~/catkin_ws/src/in_class_day02/scripts``.  We'll start out with the standard first line as well as a header comment, import the correct message type, and initialize our ROS node:

```python
#!/usr/bin/env python3
""" Investigate receiving a message using a callback function """
from geometry_msgs.msg import PointStamped
import rospy

rospy.init_node('receive_message')
```

Next, we will define our callback function.  Our callback function takes as input a single parameter which will be a Python object of the type that is being published on the topic that we subscribe to.  Eventually we will be subscribing to the topic /my_point which means that we will be writing a callback function that handles objects of type ``geometry_msgs/PointStamped``.  As a test, let's make our callback function simply print out the header attribute of the PointStamped message.

```python
def process_point(msg):
    print msg.header
```

Next, we must subscribe to the appropriate topic.

```python
rospy.Subscriber("/my_point", PointStamped, process_point)
```

The ROS runtime will take care of calling our process_point function whenever a new message is ready!  There is nothing more we have to do to make this happen!  Since ROS handles calling our callback function on a separate thread, we have to make sure to enter an infinite loop so that our main Python thread doesn't terminate (which would cause our node to exit).  To do this we'll use the rospy.spin function.

```python
rospy.spin()
```

After making your code executable (using ``chmod`` as shown earlier), try running it.  Make sure that the node we created in the first part is also running.

### Making Your Code Object-Oriented

While what we did in the previous section is a great way to gently introduce ourselves to ROS, I like to minimize the time we will practice the bad habit of not using object-oriented techniques to structure our code.

> Object-oriented programming uses the concept of objects, which are data types that contain attributes as well as methods that operate on those attributes.  As a class, let's see make a list of reasons why object-oriented principles are useful in general and why they might be useful in the context of robots specifically.

### Make Your Nodes Object-Oriented (walkthrough in main room)

Modify the code you wrote previously (``test_message.py`` and ``receive_message.py``) to be object-oriented.  There's certainly not just one way to map your code into an object-oriented structure, however, the basic principles I follow when doing this are summarized below (if you have a different way you like to do it, that's great, I'd be excited to hear about your design and the reasons you prefer it).

* Create a class that represents the node you are creating.
* The attributes of the class should represent the state of the node in question.  For instance, any value you'd like to track over the lifetime of your node can be stored as an attribute.
* The ``__init__`` method of this node should do the basic setup of the node itself (including calling ``rospy.init_node``) , creating any publishers and subscribers, and initializing any attributes.
* Callback functions should be methods of your class.  You can refer to them when setting up a Subscriber object using ``self.my_callback`` function.
* If your node has a run loop (e.g., ``test_message.py``), encapsulate that functionality in a run method of your class.

### Viewing the Results in RViz

TODO

## Work time for the Warmup Project

Take the rest of class to work on the warmup project.  Remember, you have the option of working with a partner for this project.
