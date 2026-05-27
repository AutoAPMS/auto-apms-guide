---
order: 20
---
# Getting Started

Start leveraging the advantages of Behavior Trees 🌳 fully integrated with ROS 2 🤖.

> [!NOTE]
> Currently we support **Linux only**!.

## Installation

### Using deb packages

You may install the corresponding debian packages from ROS 2's apt repository:

<!-- markdownlint-disable-next-line MD036 -->
**Core API**

```bash
sudo apt install ros-$ROS_DISTRO-auto-apms-behavior-tree-core
```

<!-- markdownlint-disable-next-line MD036 -->
**Standard Nodes & Behavior Executor**

```bash
sudo apt install ros-$ROS_DISTRO-auto-apms-behavior-tree
```

<!-- markdownlint-disable-next-line MD036 -->
**CLI Tooling**

```bash
sudo apt install ros-$ROS_DISTRO-auto-apms-ros2behavior
```

<!-- markdownlint-disable-next-line MD036 -->
**Examples**

```bash
sudo apt install ros-$ROS_DISTRO-auto-apms-examples
```

### Build from Source

As an alternative approach, you can also build from source:

1. Create a [ROS 2 workspace](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) and clone this repository

    ```bash
    mkdir ros2_ws && cd ros2_ws
    (mkdir src && cd src && git clone https://github.com/AutoAPMS/auto-apms.git)
    ```

1. Install all required dependencies. We assume that you already installed ROS 2 on your system

    ```bash
    rosdep init  # Skip this if rosdep has already been initialized
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```

1. Build and install all packages required for `auto_apms_examples`

    ```bash
    colcon build --packages-up-to auto_apms_examples --symlink-install
    ```

1. Source the workspace to update your environment

    ```bash
    source install/setup.bash
    ```

## Run your first Behavior

Run your first behavior using `ros2 behavior` (recommended). This is an extension of the ROS 2 CLI introduced by the `auto_apms_ros2behavior` package

```bash
ros2 behavior run auto_apms_examples::demo::HelloWorld --blackboard name:=Turtle
```

![auto-apms-gif](https://github.com/user-attachments/assets/0039aa09-9448-4102-9eb3-38138a805728)

## Visual Demonstration

We provide a guide for running a cool **visual demonstration** on complex behaviors created with AutoAPMS in the [auto_apms_simulation](https://github.com/AutoAPMS/auto_apms_simulation) repository. The simulation shows multiple "robots" moving in a magical hall adjusting their behavior dynamically according to the following policy: Approach the goal as long as the hallway is not occupied - if it is, retreat. *Piertotum Locomotor!*

<video autoplay controls>
    <source src="https://github.com/user-attachments/assets/c7a63a50-b064-4db2-97ab-4d81464374d6" type="video/mp4">
Your browser does not support the video tag.
</video>
