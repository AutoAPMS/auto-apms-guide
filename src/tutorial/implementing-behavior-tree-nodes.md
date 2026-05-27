---
order: 10
---
# Implementing Custom Behavior Tree Nodes

Once the robot's skill set has been defined, we must create a way to actually use them. It's common practice in ROS 2 to distribute functionality using the client-server model. Let's assume that we've already created the server of a particular function by implementing a skill using ROS's `rclcpp::Node`. The next step is to allow programs to access that skill using a specific [ROS 2 interface](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Interfaces.html) inside a corresponding client structure.

AutoAPMS advocates using [behavior trees](../concept/behavior-trees.md) for triggering skills. They are composed of nodes designed to be exactly what we search for: Function handles for specific algorithms. We provide various [standard behavior tree nodes](../reference/behavior-tree-nodes.md) that you may use for creating your behavior.

For functionality that is directly associated with custom skills, you must implement your own nodes. If your custom nodes require to use ROS 2 interfaces, you should inherit from one of the following classes:

- [`RosActionNode`](https://autoapms.github.io/auto-apms/classauto__apms__behavior__tree_1_1core_1_1RosActionNode.html) - Client node for a ROS 2 action
- [`RosServiceNode`](https://autoapms.github.io/auto-apms/classauto__apms__behavior__tree_1_1core_1_1RosServiceNode.html) - Client node for a ROS 2 service
- [`RosPublisherNode`](https://autoapms.github.io/auto-apms/classauto__apms__behavior__tree_1_1core_1_1RosPublisherNode.html) - Client node for a ROS 2 publisher
- [`RosSubscriberNode`](https://autoapms.github.io/auto-apms/classauto__apms__behavior__tree_1_1core_1_1RosSubscriberNode.html) - Client node for a ROS 2 subscriber

If you want to write nodes that are not related to any of the above mentioned ROS 2 interfaces, you should inherit from lower level classes maintained by BehaviorTree.CPP:

- [BT action node](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/include/behaviortree_cpp/action_node.h)
- [BT condition node](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/include/behaviortree_cpp/condition_node.h)
- [BT control node](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/include/behaviortree_cpp/control_node.h)
- [BT decorator node](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/include/behaviortree_cpp/decorator_node.h)

Generally speaking, every single custom behavior tree node that you implement must be derived from [`BT::TreeNode`](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/include/behaviortree_cpp/tree_node.h).

::: warning Asynchronous vs. Synchronous Nodes
While all tree nodes that are related to ROS 2 work asynchronously, other nodes may implement algorithms that do synchronous calculations. However, you should think carefully when integrating synchronous nodes and make sure that they don't prevent other nodes from being executed for too long.
:::

AutoAPMS introduces a modular, plugin-based approach for distributing node implementations within the ROS 2 workspace. The C++ source code of behavior tree nodes is supposed to be compiled to a shared library which may be loaded dynamically to instantiate the corresponding classes. For keeping track of which node belongs to which library and where it is located, we incorporate the [ament resource index](https://github.com/ament/ament_index?tab=readme-ov-file). For loading and instantiating the node classes, we utilize [pluginlib](https://wiki.ros.org/pluginlib).

We provide a straightforward approach for registering custom behavior tree nodes with the resource index and make them discoverable for `pluginlib::ClassLoader`. Our convention requires you to

1. Call the C++ macro [`AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE`](https://autoapms.github.io/auto-apms/group__auto__apms__behavior__tree.html#ga5ce6f5e1249a2f980b0487ca8bb95c08) inside the `.cpp` source file. You may call it multiple times for all your custom node classes.

2. Call the CMake macro [`auto_apms_behavior_tree_register_nodes`](../reference/cmake.md#register-nodes) inside the CMakeLists.txt of your package. You may pass multiple class names to the same call.

Here's an example:

::: code-group

```cpp [src/custom_nodes.cpp]
// Bring the base classes and the C++ macro into scope
#include "auto_apms_behavior_tree/node.hpp"

// Include custom ROS 2 interfaces
#include "my_package_interfaces/foo.hpp"

namespace my_namespace
{

// ROS 2 nodes require an interface definition as template argument
class MyCustomRosNode : public auto_apms_behavior_tree::core::RosActionNode<my_package_interfaces::action::Foo>
{
public:
  using RosActionNode::RosActionNode;

  bool setGoal(Goal& goal) override final
  {
    // Check out the API docs for more infos about what methods to override
  }
};

class MyCustomLocalNode : public BT::SyncActionNode  // Or any of the other base classes
{
public:
  using SyncActionNode::SyncActionNode;

  BT::NodeStatus tick() override final
  {
    // You must at least implement this pure virtual method
  }
};

}  // namespace my_namespace

// Make the nodes discoverable for the class loader
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(my_namespace::MyCustomRosNode) // [!code highlight:2]
AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE(my_namespace::MyCustomLocalNode)
```

```cmake [CMakeLists.txt]
project(my_package)

find_package(ament_cmake REQUIRED)
find_package(my_package_interfaces REQUIRED)
find_package(auto_apms_behavior_tree REQUIRED)

# Create a shared library that contains all your nodes
add_library(custom_nodes SHARED
  "src/custom_nodes.cpp"  # Replace with your custom path
)
target_link_libraries(custom_nodes PUBLIC
  ${my_package_interfaces_TARGETS}
  auto_apms_behavior_tree::auto_apms_behavior_tree
)

# Register your custom behavior tree nodes
auto_apms_behavior_tree_register_nodes(custom_nodes # [!code highlight:4]
  "my_namespace::MyCustomRosNode" 
  "my_namespace::MyCustomLocalNode"
)

# Install the shared library to the standard directory
install(
  TARGETS
  custom_nodes
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
```

:::

::: info What does it mean to register a node?
In the context of `auto_apms_behavior_tree` we use the term *node* as a shorthand for *behavior tree node*. **This is not to be confused with ROS 2 nodes!**

Additionally, the user needs to be aware that there is a difference between registering a node as a package resource and [registering a node with a specific `TreeDocument`](https://autoapms.github.io/auto-apms/classauto__apms__behavior__tree_1_1core_1_1TreeDocument.html#a85ea14f41726e1f6b28ade34befb570f).
:::

## About Registration Options

With the previous step, your custom nodes become available with the `ament_index` and can theoretically be instantiated when the associated shared library is loaded. However, we still need to define how exactly this should be done, since C++ behavior tree node objects are NOT supposed to be created by simply calling the constructor yourself.

If you're familiar with BehaviorTree.CPP, you'll probably know about [`BT::BehaviorTreeFactory`](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/include/behaviortree_cpp/bt_factory.h). Every behavior tree node must be instantiated using this class. To achieve that, the user is required to "register a node" using specific methods that allow for configuring the values of the arguments passed to the respective constructor. Internally, the behavior tree factory holds a map of callbacks that invoke the `BT::TreeNode::Instantiate` [factory method](https://refactoring.guru/design-patterns/factory-method). Among other things, this concept allows to decouple the low-level node implementation domain from the high-level behavior tree representation and is characteristic for model-driven software development.

When directly using `BT::BehaviorTreeFactory`, there are multiple ways of registering nodes. With AutoAPMS, we provide an alternative, unified approach for doing so by introducing so-called [node manifests](../concept/common-resources.md#behavior-tree-node-manifests). These are simple YAML files which specify the **registration options** for each node that is registered as described in the previous section. Follow the link for more information about the structure of the file.

AutoAPMS only requires the user to create a `.yaml` file, configure the node manifest and add it to the `ament_index` using certain CMake macros. No extra C++ source code is required for registering your custom nodes. This process is fully automated by the [`TreeDocument`](https://autoapms.github.io/auto-apms/classauto__apms__behavior__tree_1_1core_1_1TreeDocument.html) class. Thus, [building behavior trees](./building-behavior-trees.md) is significantly simpler when using AutoAPMS.

## Adding Node Manifests

Let's look at what the user must add to the CMakeLists.txt to provide registration options for behavior tree nodes. We may use one or more node manifest files:

::: code-group

```cmake [CMakeLists.txt (When registering nodes)]
auto_apms_behavior_tree_register_nodes(custom_nodes
  "my_namespace::MyCustomRosNode" 
  "my_namespace::MyCustomLocalNode"
  NODE_MANIFEST  # You must specify at least one [!code ++:3]
  "config/my_node_manifest.yaml"
  "config/another_node_manifest.yaml"
)
```

```cmake [CMakeLists.txt (When registering trees)]
auto_apms_behavior_tree_register_trees(
  "behavior/my_behavior_tree.xml"
  "behavior/another_behavior_tree.xml"
  NODE_MANIFEST  # You must specify at least one [!code ++:3]
  "config/my_node_manifest.yaml"
  "config/another_node_manifest.yaml"
)
```

:::

::: tip Convention 📜
We recommend storing your node manifest files under a directory called `config`. However, you can put them wherever you want, you just have to adjust the paths you pass to the CMake macros accordingly.
:::

As you can see, node manifests are relevant for both `auto_apms_behavior_tree_register_nodes` and `auto_apms_behavior_tree_register_trees`. We've already used the former macro when registering our custom node. The latter is required when registering yet another resource: The actual behavior tree source file. But more about that in another tutorial. It's pretty obvious that the resource system of ROS 2 is invaluable for AutoAPMS.

By specifying the optional `NODE_MANIFEST` keyword argument, the following steps are executed under the hood:

1. The given node manifest files are parsed and concatenated at configuration time. All classes specified using the `class_name` argument are looked up. If any of the given classes cannot be found, an error is raised.

    ::: warning No Duplicate Registration Names Allowed!
    Duplicate behavior tree nodes are not allowed, so CMake also verifies in this step that all registration names provided by the given node manifests are unique. When specifying multiple arguments under the `NODE_MANIFEST` keyword, you must make sure that the given node manifests don't use registration names more than once.
    :::

2. A [node model XML file](../concept/common-resources.md#node-model-xml-file) is generated. It will be installed under `auto_apms`/`auto_apms_behavior_tree_core`/`metadata`/`node_model_<metadata_id>.xml` respective to the package's share directory.

3. Only for `auto_apms_behavior_tree_register_nodes`: If the `NODE_MODEL_HEADER_TARGET` keyword argument is given as well, a [C++ node model header file](../concept/common-resources.md#c-node-model-header) is generated.

When any of the given node manifests or any of the associated C++ source files (i.e. the shared libraries that contain the required behavior tree node classes) are modified, steps `2.` and `3.` will be executed again the next time the package is being built. Step `1.` is executed in every build cycle.

::: tip Convention 📜
The recommended way of adding node manifests to a ROS 2 workspace is to use `auto_apms_behavior_tree_register_nodes`. With this macro, only a single metadata identifier is created and it has a concise name (same name as the target of the associated shared library). This makes it more intuitive for the user to refer to a specific manifest or the node model XML file associated with it.
:::

## Referring to Node Manifests

Once you've added node manifests using one of the methods described above, it's possible to directly refer to them using a `<manifest_alias>`. Understanding [how this identifier is determined](../concept/common-resources.md#node-manifest-identity) is crucial for the following. It is set depending on which CMake macro you call.

When using `auto_apms_behavior_tree_register_nodes`, it is determined by the name of the shared library target passed as the first positional argument.

When using `auto_apms_behavior_tree_register_trees`, the metadata is generated for each given behavior tree XML file, because there's no way of determining whether the same metadata content was generated before (that is if you pass node manifest file paths). If you want to prevent that the same metadata content is generated under different IDs or, more importantly, like to reuse previously registered node manifests, we introduce the concept of **node manifest resource identities**.

You may also use node manifest resource identities when specifying the `NODE_MANIFEST` argument, so in total there are two ways of doing so:

- Using one or more **relative paths** to YAML files.

    **This is required for initially adding resources.**

    The file paths must be relative to `CMAKE_CURRENT_SOURCE_DIR` and the content of the file must comply with the [node manifest YAML format](../concept/common-resources.md#behavior-tree-node-manifests)

- Using one or more unique **resource identities**.

    **This makes node manifests reusable.**

    The identities are formatted like `<package_name>::<manifest_alias>` and refer to specific, previously installed node manifests available with the resource index of your ROS 2 workspace.

If you're providing a resource identity but no node manifest associated with `<manifest_alias>` has been registered by the ROS 2 package `<package_name>`, CMake throws an error at configuration time.

So after successfully [adding node manifests](#adding-node-manifests) one may use resource identities to populate the `NODE_MANIFEST` argument as shown in the following snippet:

::: code-group

```cmake [CMakeLists.txt (When registering nodes)]
project(my_package)
# ...
auto_apms_behavior_tree_register_nodes(custom_nodes
  "my_namespace::MyCustomRosNode" 
  "my_namespace::MyCustomLocalNode"
  NODE_MANIFEST  # You must specify at least one
  "config/my_node_manifest.yaml"
  "config/another_node_manifest.yaml"
)
# ...
# Elsewhere inside the same CMakeLists.txt or the one of any other package
auto_apms_behavior_tree_register_nodes(more_nodes
  # ...    
  NODE_MANIFEST
  "my_package::custom_nodes" # [!code highlight]
)
```

```cmake [CMakeLists.txt (When registering trees)]
project(my_package)
# ...
auto_apms_behavior_tree_register_nodes(custom_nodes
  "my_namespace::MyCustomRosNode" 
  "my_namespace::MyCustomLocalNode"
  NODE_MANIFEST  # You must specify at least one
  "config/my_node_manifest.yaml"
  "config/another_node_manifest.yaml"
)
# ...
# Elsewhere inside the same CMakeLists.txt or the one of any other package
auto_apms_behavior_tree_register_trees(
  # ...
  NODE_MANIFEST
  "my_package::custom_nodes" # [!code highlight]
)
```

::: tip Within the same package
If you want to use nodes which have previously been registered **inside the same CMakeLists.txt**, you can conveniently write `${PROJECT_NAME}::<manifest_alias>`. So referring to the example above, it would be `${PROJECT_NAME}::custom_nodes`.
:::

If you initially passed multiple node manifest files to one of the mentioned CMake macros as it's the case with this example, they are automatically concatenated. When using the corresponding resource identity, **you implicitly refer to all manifest files given to the respective macro**.

You can mix and match these two approaches as you wish. However, already a single relative path to a YAML file requires the metadata to be generated from scratch. This means that the only way of fully reusing all associated metadata is to strictly use resource identities only.

Now that you laid the ground work for creating behavior trees, you should proceed to the next page to learn how to actually put your custom behavior tree nodes into action.
