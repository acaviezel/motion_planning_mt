# Real-time path planning for human-robot collaboration

## Prerequisites
* Clone the modified Cartesian impedance controller (forked from Curdin.)\
  [Forked Cartesian impedance Controller](https://github.com/ptliu268/cartesian_impedance_control)
* Add cartesian impedance controller to the config file located inside  `franka_moveit_config` package.\
  path: `franka_ros2/franka_moveit_config/config/panda_ros_controllers.yaml`
  ```
  cartesian_impedance_controller:
      type: cartesian_impedance_control/CartesianImpedanceController
  ```
* Also in the `franka_moveit_config` package, modify the `moveit.launch.py` launch file.
  1. Add two more key-value pair to the variable `planning_scene_monitor_parameters`.
   ```python
   planning_scene_monitor_parameters = {
      'publish_planning_scene': True,
      'publish_geometry_updates': True,
      'publish_state_updates': True,
      'publish_transforms_updates': True,
      'publish_robot_description' : True,
      'publish_robot_description_semantic' : True,
   }
   ```
   2. When loading controller, load cartesian impedance controller instead of panda_arm_controller
   ```python
    # Load controllers
    load_controllers = []
    for controller in ['joint_state_broadcaster','cartesian_impedance_controller']: # instead of panda_arm_controller
        load_controllers += [
            ExecuteProcess(
                cmd=['ros2 run controller_manager spawner {}'.format(controller)],
                shell=True,
                output='screen',
            )
        ]
   ```

## How to run the demo code
After sourcing the workspace.\
go to the root of the package.
```
cd src/motion_planning_mt
```
Launch the moveit environment and the Cartesian impedance controller.
```
ros2 launch franka_moveit_config moveit.launch.py robot_ip:=<fci-ip>
```

Run the demo scene node, which populates three moving boxes.
```
ros2 run motion_planning_mt demo_scene
```

Run the distance calculator node, which visualize the distance between the closest obstacle to the robot.
```
ros2 run motion_planning_mt distance_calculator
```

Run the local planner node.
```
ros2 run motion_planning_mt explicit_reference_governor
```

Run the global planner node.
```
ros2 run motion_planning_mt motion_planner --ros-args --params-file demo/repeat_demo.yaml
```
