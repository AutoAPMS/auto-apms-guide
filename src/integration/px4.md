---
order: 10
sidebar: PX4 Autopilot
---
# PX4 Autopilot Integration

The [`auto-apms-px4`](https://github.com/AutoAPMS/auto-apms-px4) repository connects AutoAPMS with [PX4 Autopilot](https://px4.io/). It lets a companion computer orchestrate a PX4 vehicle from behavior trees and turns behaviors into regular PX4 flight modes. Everything builds on the [PX4 ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface.html) (`px4_ros2_cpp`) and the standard `px4_msgs` topics, so no PX4 firmware changes are required.

The integration follows one rule: **PX4 always has the last word.** A behavior never writes setpoints behind the autopilot's back. It either requests one of PX4's own modes or it runs inside a mode that PX4 registered, activated and can take away again. Failsafes, RC override and the ground control station (GCS) keep working exactly as they do without AutoAPMS.

## Installation

The repository follows the branch naming of the PX4 ecosystem (`release/1.17`, ...). Check out the branch that matches your PX4 firmware, and use the same branch for [`px4_msgs`](https://github.com/PX4/px4_msgs) and [`px4-ros2-interface-lib`](https://github.com/Auterion/px4-ros2-interface-lib). The repository pins these dependencies in `dependencies.repos`, which you can import into your workspace:

```bash [Terminal]
cd ~/ros2_ws/src
git clone -b release/1.17 https://github.com/AutoAPMS/auto-apms-px4.git
vcs import < auto-apms-px4/dependencies.repos
cd .. && colcon build --packages-up-to auto_apms_px4 --symlink-install
```

The flight controller (FMU) must be connected to ROS 2 as described in the [PX4 ROS 2 user guide](https://docs.px4.io/main/en/ros2/user_guide.html), for example through the uXRCE-DDS agent. The same setup works with PX4 SITL.

The repository contains two packages:

| Package | Content |
| :--- | :--- |
| `auto_apms_px4` | Behavior tree nodes, reusable behavior trees, skills, helpers for custom modes and the behavior mode executor. |
| `auto_apms_px4_interfaces` | ROS 2 actions used by the skills (`Takeoff`, `GoTo`, `Land`, `RTL`, ...) and the `RegisteredMode` message. |

## Four Ways to Drive PX4

The integration offers four building blocks. They can be combined freely within one behavior tree.

| Building block | What it does | Extra processes | Use it when |
| :--- | :--- | :---: | :--- |
| [Vehicle command nodes](#vehicle-command-nodes) | Send `VehicleCommand` messages straight to the FMU and observe its state | None | PX4's built-in modes (takeoff, hold, land, RTL, mission, ...) cover what you need |
| [Skills](#skills) | ROS 2 action servers that run a PX4 mode per goal and report feedback and a result | Skill container | You need goal, feedback and result semantics, e.g. to go to a position and know when it is reached |
| [Custom modes](#custom-modes) | Your own `px4_ros2` modes, registered with PX4 at runtime | Your mode node | You implement your own guidance or control and want PX4 to treat it like any other mode |
| [Behavior mode executor](#behavior-mode-executor) | A PX4 mode that runs an AutoAPMS behavior when selected | Executor node | The pilot or the GCS should start the automation, e.g. from an RC switch |

### Vehicle Command Nodes

These nodes talk to the FMU directly. You only need the behavior tree executor; no additional node has to run on the companion computer.

| Node | Purpose |
| :--- | :--- |
| `SendVehicleCommand` | Publish any `VehicleCommand` (command id and `param1` to `param7`) and wait for PX4's acknowledgement. Succeeds if PX4 accepts the command, fails if PX4 rejects it or no acknowledgement arrives in time. |
| `SendCmdSetNavState` | Switch to the flight mode given by its `nav_state`. |
| `SendCmdDoReposition` | Fly to a global WGS84 position using PX4's reposition command. |
| `CheckNavState` | Succeed if the given mode is active. |
| `CheckArmingState` | Succeed if the vehicle is armed (`2`) or disarmed (`1`). |
| `CheckModeCompleted` | Succeed once the given mode reports that it finished, e.g. that the landing is complete. |
| `GetModeNavState` | Resolve a mode name to its `nav_state`, including custom modes registered at runtime. |
| `GetGlobalPosition`, `GetLocalPosition` | Write the current position to the blackboard. |
| `OffsetGlobalPosition` | Offset a global position by a north, east, down vector, e.g. to compute a reposition target. |

Because every command waits for its acknowledgement, a command that PX4 silently rejects makes the node fail instead of leaving the tree waiting forever.

The package registers a set of reusable trees built from these nodes under the resource `auto_apms_px4::vehicle_command`. They follow a simple naming convention: `Set*` trees request a state and return once PX4 reflects it, `Do*` trees perform a maneuver and return once PX4 reports that it completed.

| Tree | Effect |
| :--- | :--- |
| `SetArm`, `SetDisarm` | Arm or disarm and wait for the arming state. |
| `SetModeHold`, `SetModeRTL`, `SetModeLand`, `SetModeTakeoff`, `SetModePosition`, `SetModeMission` | Switch to the mode and wait until it is active. |
| `SwitchModeAndWait` | Switch to the mode given by the `nav_state` port and wait until it is active. |
| `SwitchToNamedModeAndWait` | Same, but resolve the mode by name first. Use this for custom modes. |
| `SetGlobalGoTo` | Reposition to `lat`, `lon`, `alt` (and optionally `yaw`) without waiting for arrival. |
| `SetPx4Param` | Set a PX4 parameter by its index (blackboard parameters `bb.param_number` and `bb.param_value`). |
| `DoTakeoff`, `DoLanding` | Take off or land and wait until PX4 reports completion. |

You can reuse these trees in your own behaviors with the [`<include>` element](../tutorial/building-behavior-trees.md#including-other-behavior-trees). The include picks only the named tree and the trees it depends on:

::: code-group

```xml [behavior/takeoff_and_land.xml]
<root BTCPP_format="4" main_tree_to_execute="TakeoffAndLand">
  <include autoapms="auto_apms_px4::vehicle_command::SetArm"/>
  <include autoapms="auto_apms_px4::vehicle_command::DoTakeoff"/>
  <include autoapms="auto_apms_px4::vehicle_command::DoLanding"/>

  <BehaviorTree ID="TakeoffAndLand">
    <Sequence>
      <SubTree ID="SetArm"/>
      <SubTree ID="DoTakeoff"/>
      <SubTree ID="DoLanding"/>
    </Sequence>
  </BehaviorTree>
</root>
```

```cmake [CMakeLists.txt]
auto_apms_behavior_tree_register_trees(
  "behavior/takeoff_and_land.xml"
  NODE_MANIFEST
  "auto_apms_behavior_tree::behavior_tree_nodes"
  "auto_apms_px4::behavior_tree_nodes"
)
```

```bash [Terminal]
ros2 behavior run my_package::takeoff_and_land::TakeoffAndLand
```

:::

`auto_apms_px4` itself registers a very similar tree as `auto_apms_px4::example::TakeoffAndLand`, which you can run right away against a simulated vehicle.

### Skills

Skills wrap PX4 modes as ROS 2 action servers. When a skill receives a goal, it activates its mode, publishes feedback while the mode runs and returns a result once the mode completes or fails. Start all skills in one component container:

```bash [Terminal]
ros2 launch auto_apms_px4 skills_launch.py namespace:=my_vehicle
```

| Action name | Interface | Effect |
| :--- | :--- | :--- |
| `arm_disarm` | `auto_apms_px4_interfaces/action/ArmDisarm` | Arm or disarm, optionally waiting until the vehicle is ready to arm |
| `takeoff` | `auto_apms_px4_interfaces/action/Takeoff` | Take off to an altitude |
| `go_to_global`, `go_to_local` | `auto_apms_px4_interfaces/action/GoTo` | Fly to a global or local position with velocity limits and arrival thresholds |
| `enable_hold` | `auto_apms_px4_interfaces/action/EnableHold` | Hold the current position |
| `land` | `auto_apms_px4_interfaces/action/Land` | Land at the current position |
| `rtl` | `auto_apms_px4_interfaces/action/RTL` | Return to launch |
| `set_actuators` | `auto_apms_px4_interfaces/action/SetActuators` | Command motors and servos directly |

The behavior tree nodes `GoTo`, `GoToLocal`, `GoToGlobal` and `SetActuators` are the matching clients. `GoTo` selects the action server through its `frame` port, so the same node serves both `go_to_local` and `go_to_global`. The tree `auto_apms_px4::relative_goto::RelativeGoTo` shows how to combine `GetLocalPosition` with `GoTo` to fly relative to the current position.

Since every skill is an ordinary ROS 2 action, you can also call it from C++ or Python code that does not use behavior trees at all. `auto_apms_px4` ships a small example (`takeoff_and_land_example`) that does exactly that.

### Custom Modes

If PX4's built-in modes are not enough, you write your own mode as a subclass of `px4_ros2::ModeBase` and let `auto_apms_px4` handle the registration. The helpers wait for the FMU, register the mode and announce its dynamically assigned `nav_state` on the `registered_modes` topic, so behaviors can find the mode by name:

| Helper | Your mode derives from | The mode is activated by |
| :--- | :--- | :--- |
| `ModeRegistrationFactory<ModeT>` | `px4_ros2::ModeBase` | Switching to its `nav_state` (from a behavior tree, an RC switch or the GCS) |
| `ModeProxyActionFactory<ActionT, ModeT>` | `auto_apms_px4::ActionDrivenMode<ActionT>` | Sending a goal to the action server the factory creates, i.e. the mode becomes a [skill](#skills) |

A plain mode looks like this:

```cpp [my_mode.cpp]
#include "auto_apms_px4/mode_registration.hpp"
#include "px4_ros2/components/mode.hpp"

namespace my_pkg
{

class MyMode : public px4_ros2::ModeBase
{
public:
  MyMode(rclcpp::Node & node, const px4_ros2::ModeBase::Settings & settings) : px4_ros2::ModeBase(node, settings)
  {
    // Create your px4_ros2 setpoint types here
  }

  void onActivate() override {}
  void onDeactivate() override {}
  void updateSetpoint(float dt_s) override { /* Publish your setpoints */ }
};

class MyModeComponent : public auto_apms_px4::ModeRegistrationFactory<MyMode>
{
public:
  explicit MyModeComponent(const rclcpp::NodeOptions & options) : ModeRegistrationFactory("My Mode", options) {}
};

}  // namespace my_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_pkg::MyModeComponent)
```

Once the node runs, the mode shows up in the GCS like any other mode, and a behavior tree switches to it by name:

```xml
<include autoapms="auto_apms_px4::vehicle_command::SwitchToNamedModeAndWait"/>
<!-- ... -->
<SubTree ID="SwitchToNamedModeAndWait" mode_name="My Mode"/>
```

The [repository README](https://github.com/AutoAPMS/auto-apms-px4#writing-custom-px4-modes) shows the action-driven variant. The skill sources in `auto_apms_px4/src/skill/` are complete examples of it.

### Behavior Mode Executor

The behavior mode executor turns an AutoAPMS behavior into a PX4 mode. It registers a mode (named `AutoAPMS Behavior` by default) with PX4. When this mode is selected, PX4 puts the executor in charge and the executor runs the behavior. This way the automation is started from PX4 itself: by an RC switch, from the GCS, or immediately after startup.

```bash [Terminal]
ros2 launch auto_apms_px4 behavior_mode_executor_launch.py \
  behavior:=auto_apms_px4::example::TakeoffAndLand
```

The executor builds the behavior tree ahead of time, so activating the mode only starts ticking an existing tree. A pilot override or a failsafe deactivates the mode and terminates the behavior. When the behavior ends, the executor performs a configurable reaction and prepares a fresh tree for the next activation.

| Parameter | Default | Changeable at runtime | Description |
| :--- | :---: | :---: | :--- |
| `activation` | `armed` | No | When PX4 may activate the executor: `armed`, `always` or `immediately`. |
| `mode_name` | `AutoAPMS Behavior` | No | Name of the mode as shown in the GCS. |
| `on_completion` | `hold` | Yes | Reaction after the behavior succeeds: `hold`, `rtl`, `land`, `disarm`, `complete` (hand control back to PX4) or `none`. |
| `on_failure` | `hold` | Yes | Reaction after the behavior fails (same options). |
| `defer_failsafes` | `false` | Yes | Defer PX4 failsafes while the behavior runs. Use with care. |
| `behavior.build_request`, `behavior.entry_point`, `behavior.node_manifest` | | No | The behavior to prepare on startup. |

Changes to the dynamic parameters take effect the next time the executor is put in charge. The behavior itself is changed through two interfaces. Both first build the new behavior tree and only accept it if that succeeds, so a broken behavior never replaces a working one:

```bash [Terminal]
# Replace the behavior without running it
ros2 service call /behavior_mode_executor/set_behavior auto_apms_interfaces/srv/SetBehavior \
  "{build_request: 'my_package::takeoff_and_land::TakeoffAndLand'}"

# Replace the behavior and start it: the executor requests its mode and waits until PX4 activates it
ros2 behavior send /behavior_mode_executor/start my_package::takeoff_and_land::TakeoffAndLand
```

While the executor is in charge, the vehicle command nodes send their commands on PX4's mode executor command topic and on behalf of the executor. PX4 therefore attributes a mode switch requested by the behavior to the executor itself, and the executor stays in charge.

::: tip Built on a generic executor
The behavior mode executor is not a special case of AutoAPMS. It embeds the regular [`GenericTreeExecutorNode`](../concept/behavior-executor.md#executor-classes) inside a `px4_ros2::ModeExecutorBase` and adds its own parameters to the same ROS 2 node. If you need an executor that is triggered by something other than PX4, the same pattern applies.
:::

## Example: Inspection Flight With a Safe Fallback

The following behavior takes off, flies a square at 10 m altitude using the `GoToLocal` skill and lands. If any step fails, the vehicle returns to launch. The skills must be running.

```xml [behavior/square.xml]
<root BTCPP_format="4" main_tree_to_execute="FlySquare">
  <include autoapms="auto_apms_px4::vehicle_command::SetArm"/>
  <include autoapms="auto_apms_px4::vehicle_command::DoTakeoff"/>
  <include autoapms="auto_apms_px4::vehicle_command::DoLanding"/>
  <include autoapms="auto_apms_px4::vehicle_command::SetModeRTL"/>

  <BehaviorTree ID="FlySquare">
    <Fallback>
      <Sequence>
        <SubTree ID="SetArm"/>
        <SubTree ID="DoTakeoff"/>
        <GoToLocal vector="10;0;-10"/>
        <GoToLocal vector="10;10;-10"/>
        <GoToLocal vector="0;10;-10"/>
        <GoToLocal vector="0;0;-10"/>
        <SubTree ID="DoLanding"/>
      </Sequence>
      <SubTree ID="SetModeRTL"/>
    </Fallback>
  </BehaviorTree>
</root>
```

Run it standalone with `ros2 behavior run`, or hand it to the behavior mode executor so that the pilot starts it with an RC switch and can take over at any time. The behavior is identical in both cases.

## Behavior Tree Nodes

The node manifest `auto_apms_px4::behavior_tree_nodes` registers all nodes of this integration. Pass it to `auto_apms_behavior_tree_register_trees` as shown above, or derive your own registrations from it using the [`parent` field](../concept/common-resources.md#behavior-tree-node-manifests). The [node reference](../reference/behavior-tree-nodes.md#overview-auto-apms-px4) lists all ports.
