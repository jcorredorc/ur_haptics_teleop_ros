# Incluir gripper en el modelo ur3

Ver carpeta [gripper_description/](../gripper_description/)

El procedimiento para incluir el effector final al robot UR3 se toma de [2]






```
$rosrun xacro xacro --inorder -o ur3_with_vacuum_gripper.urdf ur3_with_vacuum_gripper.xacro
```

```
$check_urdf ur3_with_vacuum_gripper.urdf 

robot name is: ur3_with_vacuum_gripper
---------- Successfully Parsed XML ---------------
root Link: world has 1 child(ren)
    child(1):  base_link
        child(1):  base
        child(2):  shoulder_link
            child(1):  upper_arm_link
                child(1):  forearm_link
                    child(1):  wrist_1_link
                        child(1):  wrist_2_link
                            child(1):  wrist_3_link
                                child(1):  ee_link
                                child(2):  tool0
                                    child(1):  vacuum_gripper
                                        child(1):  tcp
```

Ver el robot y el gripper en Rviz:

```
$ roslaunch ur_haptics_teleop_ros display_ur3_gripper.launch
```

# References:

[1] https://github.com/trabelsim/UR3_with_Robotiq_gripper_85
[2] https://gramaziokohler.github.io/compas_fab/0.21.0/examples/03_backends_ros/07_ros_create_urdf_ur5_with_measurement_tool.html