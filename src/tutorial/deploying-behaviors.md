---
order: 30
---
# Deploying Behaviors

In this tutorial we're going to describe everything that you need to know when using AutoAPMS's [behavior tree executor](../concept/behavior-executor.md).

For deploying a behavior you need to [build a behavior tree](./building-behavior-trees.md) beforehand. Read the corresponding tutorial if you're not familiar with this process.

## Executing a Behavior

The package `auto_apms_behavior_tree` offers an executable called `run_behavior` which automatically spawns the executor and starts executing a specific behavior in one go. It implements a custom runtime and handles SIGINT signals appropriately. Use it like this:

::: code-group

```bash [Terminal]
ros2 run auto_apms_behavior_tree run_behavior -h  # Prints help information
```

```py [launch.py]
from launch import LaunchDescription
from auto_apms_behavior_tree.launch import RunBehavior

def generate_launch_description():
    return LaunchDescription(
        [
            RunBehavior(
                # ...
            )
        ]
    )
```

:::

It accepts a `build_request`, `entry_point` and a `node_manifest` as arguments according to the [behavior definition](../concept/fundamentals.md#understanding-behaviors). By default, you can simply pass a [behavior tree resource identity](../concept/common-resources.md#behavior-trees) and the corresponding behavior tree will be executed immediately. This is because the behavior executor loads `TreeFromResourceBuildHandler` on startup if the user doesn't overwrite the default. You can specify a different build handler by setting the corresponding ROS 2 parameter like this:

::: code-group

```bash [Terminal]
ros2 run auto_apms_behavior_tree run_behavior "<build_request>" --ros-args -p build_handler:=my_namespace::MyBuildHandlerClass
```

```py [launch.py]
from launch import LaunchDescription
from launch_ros.actions import Node
from auto_apms_behavior_tree.launch import RunBehavior

def generate_launch_description():
    return LaunchDescription(
        [
            RunBehavior(
                build_request="<build_request>",
                build_handler="my_namespace::MyBuildHandlerClass"
            ),
        ]
    )
```

:::

::: info
If you cannot use `run_behavior` for some reason (e.g. when applying ROS 2 composition), you should first spawn the node and then, in a separate step, send a [`StartTreeExecutor`](../concept/behavior-executor.md#starttreeexecutor) action goal.

As a last resort, you can also pass a build request as the first command line argument of `tree_executor` which populates the `arguments` member of `rclcpp::NodeOptions`. This will also cause the executor to automatically start ticking the behavior tree but the runtime doesn't handle interrupt signals very well.
:::

We also offer a package that extends the ROS 2 command line interface with a subcommand specifically dedicated to deploying behaviors. It wraps the `run_behavior` executable and offers tab completion. You simply need to build `auto_apms_ros2behavior` and source your workspace again. Afterwards, you have access to the [`ros2behavior` CLI extension](../reference/ros2behavior.md) which allows you to execute a behavior like this:

```bash [Terminal]
ros2 behavior run -h  # Prints help information
```

**This is the recommended way of executing behaviors from the command line** since it integrates very well with the other concepts introduced by AutoAPMS and looks the cleanest.

## Spinning the Executor

Users may also spawn the behavior tree executor as a persistent ROS 2 node. Compared to running a behavior using a "one-shot" CLI tool, spawning the executor node has the advantage that we can dynamically execute a behavior during runtime e.g. from other ROS 2 nodes. We provide an executable called `tree_executor` which spawns the executor and spins the ROS 2 node until the process is interrupted. You may run it in the terminal or a [ROS 2 launch file](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html) like this:

::: code-group

```bash [Terminal]
ros2 run auto_apms_behavior_tree tree_executor
```

```py [launch.py]
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="auto_apms_behavior_tree",
                executable="tree_executor"
            )
        ]
    )
```

:::

If you're using [ROS 2 Composition](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Composition.html), the corresponding component can be found under the name `auto_apms_behavior_tree::TreeExecutorNode` and spawned like this:

::: code-group

```bash [Terminal]
# Spawn the component container
ros2 run rclcpp_components component_container

# Add the executor node to the container (from another terminal)
ros2 component load /ComponentManager auto_apms_behavior_tree auto_apms_behavior_tree::TreeExecutorNode
```

```py [launch.py]
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription(
        [
            ComposableNodeContainer(
                name="my_container",
                namespace="my_namespace",
                package="rclcpp_components",
                executable="component_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="auto_apms_behavior_tree", 
                        plugin="auto_apms_behavior_tree::TreeExecutorNode"
                    )
                ]
            )
        ]
    )
```

:::

## Interacting with the Executor

Users may communicate with a spinning executor node by creating ROS 2 action clients and sending goals to the corresponding actions:

- [`StartTreeExecutor`](../concept/behavior-executor.md#starttreeexecutor)
- [`CommandTreeExecutor`](../concept/behavior-executor.md#commandtreeexecutor)

The former action allows to initiate the execution of a particular behavior tree and the latter offers a way to interrupt/restart this process.

We conveniently provide behavior tree nodes associated with these actions:

- [`StartExecutor`](../reference/behavior-tree-nodes.md#startexecutor)
- [`ResumeExecutor`](../reference/behavior-tree-nodes.md#resumeexecutor)
- [`PauseExecutor`](../reference/behavior-tree-nodes.md#pauseexecutor)
- [`HaltExecutor`](../reference/behavior-tree-nodes.md#haltexecutor)
- [`TerminateExecutor`](../reference/behavior-tree-nodes.md#terminateexecutor)

Since a behavior tree executor implements the same standard interfaces as any other ROS 2 node, it's also possible to query or manipulate parameters at runtime. Besides the static [executor parameters](../concept/behavior-executor.md#configuration-parameters) you may additionally use [scripting enum parameters](../concept/behavior-executor.md#scripting-enums) or [global blackboard parameters](../concept/behavior-executor.md#global-blackboard).

The following nodes allow for interacting with ROS 2 parameters from within a behavior tree:

- [`HasParameter`](../reference/behavior-tree-nodes.md#hasparameter)
- [`GetParameter`](../reference/behavior-tree-nodes.md#getparameter)
- `GetParameter<TypeName>` (implemented for each ROS 2 parameter type)
- [`SetParameter`](../reference/behavior-tree-nodes.md#setparameter)
- `SetParameter<TypeName>` (implemented for each ROS 2 parameter type)

## Using a Suitable Build Handler

To define how the executor builds the behavior tree to be executed, you must tell it which build handler implementation to use. This can be done by in two ways:

- Setting the executor's ROS 2 parameter named `build_handler`.
- Using the `build_handler` field of a [`StartTreeExecutor`](../concept/behavior-executor.md#starttreeexecutor) action goal.

The user is expected to provide a resource identity in order to specify the build handler to be used. Visit [this page](../concept/common-resources.md#behavior-build-handlers) to learn more about build handler resources and the standard implementations we provide.

## Nesting Behaviors

The flexible action interface introduced by the behavior tree executor makes it possible to deploy behaviors remotely. This architecture also allows for multiple behavior trees being executed in real parallelism by different executor nodes communicating with each other. Therefore, this functionality empowers users of AutoAPMS to design arbitrarily complex systems and spawn deeply nested behaviors trees.

By using `StartExecutor` and the other behavior tree nodes designed for interacting with `TreeExecutorNode`, the user is not limited to including subtrees locally but profits significantly from increased modularity. With the `attach` field, you can control whether to attach to a behavior tree and wait for it to be completed or detach one behavior from another. In detached mode, you're able to do work asynchronously but must manually make sure to not leave the remote executor running after the parent behavior completed (for example by ticking `TerminateExecutor` last thing).

The next tutorial explains how nested behaviors are used in AutoAPMS to create reactive behaviors. We specifically organize multiple behavior trees and create an even higher level of abstraction: Missions.
