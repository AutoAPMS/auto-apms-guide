---
order: 20
sidebar: ros2_control
---
# ros2_control Integration

[ros2_control](https://control.ros.org) is the standard ROS 2 framework for real-time control. Its controller manager loads hardware components and controllers as plugins, runs them in a real-time loop and lets you switch controllers at runtime through ROS 2 services. The [`auto_apms_ros2control`](https://github.com/AutoAPMS/auto_apms_ros2control) package provides behavior tree nodes for all of these services, plus conditions that observe controllers, hardware components and interface values.

This makes controller management part of your behaviors. A behavior can bring up the hardware in a defined order, change the control mode in the middle of a task, wait until a controller is active before it moves on, or abort a motion when a measured value leaves its allowed range. The control loop itself stays inside ros2_control; the behavior tree only decides which controllers run and when.

## Installation

Clone the repository into your workspace and build it:

```bash [Terminal]
cd ~/ros2_ws/src
git clone https://github.com/AutoAPMS/auto_apms_ros2control.git
cd .. && rosdep install --from-paths src --ignore-src -y
colcon build --packages-up-to auto_apms_ros2control --symlink-install
```

The package depends only on the message definitions of ros2_control (`controller_manager_msgs`, `lifecycle_msgs` and `pal_statistics_msgs`), not on the controller manager itself. The behavior tree executor can therefore run on a different machine than the control loop, as long as both share the same ROS 2 network.

## Behavior Tree Nodes

All nodes are registered by the node manifest `auto_apms_ros2control::behavior_tree_nodes`.

| Category | Nodes | Purpose |
| :--- | :--- | :--- |
| Switching controllers | `SwitchController`, `ActivateController`, `DeactivateController` | Activate and deactivate controllers. `SwitchController` does both in the same update cycle of the controller manager. |
| Controller lifecycle | `LoadController`, `ConfigureController`, `CleanupController`, `UnloadController`, `ReloadControllerLibraries` | Move a controller through its lifecycle without activating it. |
| Hardware lifecycle | `SetHardwareComponentState` | Move a hardware component to `unconfigured`, `inactive`, `active` or `finalized`. |
| Queries | `ListControllers`, `ListControllerTypes`, `ListHardwareComponents`, `ListHardwareInterfaces` | Write names and states to the blackboard, e.g. all active controllers or all claimed command interfaces. |
| Conditions | `CheckControllerState`, `CheckHardwareState`, `CheckInterfaceValue` | Succeed if a controller or hardware component is in the expected state, or if an interface value lies within a range. |

Ports that take several controller names expect a `;`-separated list, for example `activate_controllers="arm_controller;gripper_controller"`. The [node reference](../reference/behavior-tree-nodes.md#overview-auto-apms-ros2control) lists all ports.

### Addressing the Controller Manager

Every node has a `controller_manager` input port (default: `controller_manager`). The node derives the service or topic name from it, so you never have to spell out names like `/controller_manager/switch_controller`. The usual ROS 2 name expansion rules apply:

| Value | Resolves to |
| :--- | :--- |
| `controller_manager` | `<executor namespace>/controller_manager`, i.e. the controller manager of the same robot |
| `/robot2/controller_manager` | Exactly this controller manager, regardless of the executor's namespace |

The relative default fits the common setup of one executor and one controller manager per robot in the same namespace. Start the executor in the robot's namespace and the same behavior works for every robot:

```bash [Terminal]
ros2 behavior run my_package::my_tree::MyTree --namespace robot1
ros2 behavior run my_package::my_tree::MyTree --namespace robot2
```

### Deriving Your Own Nodes

If your behaviors refer to the same controllers again and again, derive dedicated nodes in your own node manifest using the [`parent` field](../concept/common-resources.md#behavior-tree-node-manifests). The derived node inherits everything from its parent and only overrides what you specify:

```yaml [config/node_manifest.yaml]
ActivateArmController:
  parent: auto_apms_ros2control::behavior_tree_nodes.ActivateController
  port_default:
    activate_controllers: arm_controller

IsGripperReady:
  parent: auto_apms_ros2control::behavior_tree_nodes.CheckHardwareState
  port_default:
    hardware: gripper
    state: active
```

The trees then read like a description of the task rather than a list of service calls: `<ActivateArmController/>`, `<IsGripperReady/>`.

### Monitoring Interface Values

`CheckInterfaceValue` reads the value of any state, command or reference interface and succeeds if it lies within `[min, max]` (both bounds are optional). Interface names follow the naming of the controller manager's [introspection](https://control.ros.org/jazzy/doc/ros2_control/doc/introspection.html), for example `state_interface.joint1/position` or `command_interface.joint1/velocity`. The current value is also written to the `value` output port.

The node subscribes to `<controller_manager>/introspection_data/full`. It fails if the interface is not part of the introspection data, or if no new data arrived within `timeout` seconds (default 0.5 s). A controller manager that stops publishing therefore never passes the check.

## Use Cases

### Bringing Up a Robot in a Defined Order

Spawner scripts in launch files start controllers as soon as they can, without knowing whether the hardware is ready. A behavior can make the order explicit and fail early with a clear reason:

```xml [behavior/bringup.xml]
<BehaviorTree ID="Bringup">
  <Sequence>
    <SetHardwareComponentState component="arm" target_state="active"/>
    <RetryUntilSuccessful num_attempts="10">
      <CheckHardwareState hardware="arm" state="active"/>
    </RetryUntilSuccessful>
    <LoadController controller="joint_state_broadcaster"/>
    <ConfigureController controller="joint_state_broadcaster"/>
    <LoadController controller="arm_controller"/>
    <ConfigureController controller="arm_controller"/>
    <SwitchController activate_controllers="joint_state_broadcaster;arm_controller"/>
  </Sequence>
</BehaviorTree>
```

`LoadController` requires the controller's type to be set in the controller manager's parameters (`<controller_name>.type`), just as the spawner does.

### Changing the Control Mode During a Task

Many tasks need different controllers in different phases, for example a trajectory controller for free motion and an admittance or force controller once the tool touches a surface. `SwitchController` deactivates the old and activates the new controller in the same update cycle, so the hardware is never left without a controller in between:

```xml [behavior/contact_task.xml]
<BehaviorTree ID="ContactTask">
  <Sequence>
    <!-- Free motion with the trajectory controller -->
    <MoveToApproachPose/>
    <SwitchController activate_controllers="admittance_controller"
                      deactivate_controllers="trajectory_controller"/>
    <!-- Contact phase -->
    <PressAlongSurface/>
    <SwitchController activate_controllers="trajectory_controller"
                      deactivate_controllers="admittance_controller"/>
  </Sequence>
</BehaviorTree>
```

`MoveToApproachPose` and `PressAlongSurface` stand for your own nodes, for example a `RosActionNode` that sends a trajectory goal. With the default `strictness="STRICT"`, the switch fails as a whole if one of the controllers cannot be switched, and the behavior can react to that failure.

### Guarding a Motion With a Measured Value

Combined with a `ReactiveSequence`, `CheckInterfaceValue` becomes a guard that is evaluated on every tick while the motion is running. If the measured joint effort leaves the allowed range, the motion is halted and the fallback switches to a controller that holds the current position:

```xml [behavior/guarded_motion.xml]
<BehaviorTree ID="GuardedMotion">
  <Fallback>
    <ReactiveSequence>
      <CheckInterfaceValue interface="state_interface.joint2/effort" min="-40" max="40"/>
      <MoveToApproachPose/>
    </ReactiveSequence>
    <SwitchController activate_controllers="hold_position_controller"
                      deactivate_controllers="trajectory_controller"
                      strictness="BEST_EFFORT"/>
  </Fallback>
</BehaviorTree>
```

This guard runs at the tick rate of the behavior tree, not at the rate of the control loop. Use it for supervision at the task level; limits that must hold within one control cycle belong into the controller or the hardware component.

## Trying Nodes From the Command Line

Each node can be run on its own with [`ros2 behavior node call`](../reference/ros2behavior.md#ros2-behavior-node-call). This is useful to check the names of controllers and interfaces before you put them into a tree:

```bash [Terminal]
ros2 behavior node call auto_apms_ros2control::behavior_tree_nodes ListControllers --namespace robot1
ros2 behavior node call auto_apms_ros2control::behavior_tree_nodes CheckInterfaceValue \
  interface:=state_interface.joint2/effort min:=-40 max:=40
```
