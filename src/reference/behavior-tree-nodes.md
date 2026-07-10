---
order: 20
sidebar: Behavior Tree Nodes
---
# Behavior Tree Node Reference

Here is an extensive list of all behavior tree nodes that are available and ready to use with AutoAPMS.

We also include the documentation of those nodes that natively come with [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) so you have a better overview.

There's a [C++ node model class](../concept/common-resources.md#behavior-tree-node-models) available for all of the registration names mentioned below. You can include them like so:

```cpp
#include "auto_apms_behavior_tree/behavior_tree_nodes.hpp"
namespace standard_node_models = auto_apms_behavior_tree::model;

#include "auto_apms_simulation/behavior_tree_nodes.hpp"
namespace simulation_node_models = auto_apms_simulation::model;

#include "auto_apms_px4_behavior/behavior_tree_nodes.hpp"
namespace px4_node_models = auto_apms_px4_behavior::model;
```

## Overview `auto-apms`

<!--@include: ./create_node_reference_markdown_output.md-->

## Overview [`auto_apms_ros2control`](https://github.com/AutoAPMS/auto_apms_ros2control)

<!--@include: ./create_node_reference_markdown_output_ros2control.md-->

## Overview [`auto-apms-px4`](https://github.com/AutoAPMS/auto-apms-px4)

<!--@include: ./create_node_reference_markdown_output_px4.md-->
