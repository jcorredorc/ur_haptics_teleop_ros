
# Install falcon haptic device drive and ros package

1. Install the driver

```
git clone https://github.com/libnifalcon/libnifalcon

cd libnifalcon

mkdir build
cd build
cmake -G "Unix Makefiles" ..
make
sudo make install

```

2. Test the device

Connect the device to the computer then run the following commands with sudo,

```
findfalcons 
```

Some apps to test the device

```
falcon_test_cli --cube_test | --sphere_test  | --z_wall_test

falcon_led --led_red # it doesn't work
```


# Install the ros_falcon package


1. Clone the repo

```
git clone https://github.com/jcorredorc/ros_falcon.git
```

2. Install joy package if required 

```
sudo apt install ros-noetic-joy 
```

3. Install ros_falcon package, 

```
catkin build
```

4. Setting the udev permissions,

```
roscd ros_falcon
sudo cp udev_rules/99-udev-novint.rules /etc/udev/rules.d
```

Now you shouldn't need use sudo to test the falcon,

```
findfalcons 
```

5. Test the ros node,

```
rosrun ros_falcon falcon_node 
```


```
$ rosnode info /ROSfalcon 
--------------------------------------------------------------------------------
Node [/ROSfalcon]
Publications: 
 * /falconPos [ros_falcon/falconPos]
 * /rosout [rosgraph_msgs/Log]

Subscriptions: 
 * /falconForce [unknown type]

Services: 
 * /ROSfalcon/get_loggers
 * /ROSfalcon/set_logger_level
```


watch  /falconPos

```
$ rostopic echo -n1 /falconPos 
X: 0.010844005985708479
Y: -0.05195627572281342
Z: 0.0819370307925486
---
```


send a force to falcon,

```
 rostopic pub /falconForce ros_falcon/falconForces '{X: 0, Y: 0.0, Z: -2}'
```

> May the force be with you ! :P
