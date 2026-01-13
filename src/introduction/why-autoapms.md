---
order: 10
---
# Why AutoAPMS?

As you might already know, designing a robotic real-time system and enabling it to manipulate or move around its environment is a painfully time consuming task and requires a lot of practical experience. Shortly after you started prototyping, you'll probably realize that a big portion of what you do isn't actually related to your application. Especially when it becomes more complex, there are a lot of "side quests" which revolve around structuring functionality, maintaining modularity and most importantly defining operational fallback mechanisms if something goes wrong.

"If only there was another one of those helpful ROS 2 packages to provide me with a framework to deal with all that", you might think. 🤔

Enter **AutoAPMS**: A ROS 2 project that offers a unified toolset and a flexible system architecture designed for helping you focus on the most important development tasks. It provides an end-to-end solution for designing and executing robotic applications.

## Use Case

AutoAPMS's modular design allows software components to be reused effectively and introduces powerful concepts for creating behaviors from a high level of abstraction. This software adds a top-level deliberation layer to existing mission control solutions provided by [Nav2](https://nav2.org/) or [PX4](https://px4.io/). These kinds of software stacks commonly provide functionality on the execution level. AutoAPMS, however, is designed to orchestrate the underlying processes from the management level, effectively increasing the intelligence of an autonomous system and therefore it's capability to operate resiliently.

However, we do not offer a full-fledged planning system like [AIPlan4EU](https://github.com/aiplan4eu/unified-planning) and [PlanSys2](https://github.com/PlanSys2/ros2_planning_system) for example. Instead, we mostly provide user-oriented development tools which facilitate implementing the underlying actions/tasks and manually composing them to achieve intermediate objectives. One use case would be to assemble individual procedures using AutoAPMS and then formulate a mission plan employing a domain-specific algorithm provided by other projects. It's up to the user to define the degree of complexity for procedures implemented with AutoAPMS. If your use case is rather simple or you prefer a more comprehensible way of planning a mission, we additionally offer functionality that allows to manually configure behaviors to represent bigger missions. Refer to the page about AutoAPMS's [Mission Architecture](../concept/mission-architecture.md) for more info.

::: info
The long-term goal is to eventually be able to incorporate existing planning libraries when applying our mission architecture. We want robots to master arbitrary complex problems while being able to dynamically react to contingencies/emergencies in a sensible way.
:::

## Features

AutoAPMS provides developers of real-time systems with the following key features:

- Convenient resource management using `ament_cmake` and `ament_index`

- Inherently extensible due to plugin-based design

- Flexible and highly configurable behavior execution engine

- Powerful C++ behavior tree builder API (a supplement to BehaviorTree.CPP)

- High-level node manifests for registering node plugins without writing a single line of code

- Support for custom behavior definitions and tree builder algorithms

- `ros2 behavior` command extending the ROS2 CLI for behavior management

These features frame a unified toolset for implementing behavior-based systems and make it significantly more user-friendly and less error prone to design and execute robotic applications.

## What makes it special?

Behavior trees have also been adopted for creating robotic applications by e.g. [Nav2 Behavior Tree](https://docs.nav2.org/behavior_trees/index.html). Compared to the `nav2_behavior_tree` package, AutoAPMS thinks behavior trees further. The entire project is dedicated to simplify behavior development and automates/standardizes most of the troublesome and error-prone manual development work.

This project wraps the functionality of [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) and fully integrates it into the ROS 2 ecosystem in a lightweight manner. This way, we can leverage the advantages of behavior trees while keeping the overhead low. The core packages are written in C++ for performance reasons, but a supplementary Python API exposes high-level features for scripting.
