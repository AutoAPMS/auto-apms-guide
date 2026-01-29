---
order: 10
sidebar: CMake
---
# CMake Reference
<!-- markdownlint-disable MD024 -->  

AutoAPMS offers the following CMake macros for convenience when configuring the CMakeLists.txt of a ROS 2 package.

## `auto_apms_behavior_tree_register_nodes` {#register-nodes}

**Add behavior tree node plugins to the resource index.**

This macro must be called to make behavior tree node plugins available at runtime and configure their registration callbacks with the behavior tree factory. Optionally, a corresponding node model header is generated. This header facilitates integrating the specified nodes when building behavior trees using [`TreeDocument`](https://autoapms.github.io/auto-apms/classauto__apms__behavior__tree_1_1core_1_1TreeDocument.html).

### Signature

```cmake
auto_apms_behavior_tree_register_nodes(<target> <class_names>...
  [NODE_MANIFEST node_manifest1 [node_manifest2 ...]]
  [NODE_MODEL_HEADER_TARGET header_target]
)
```

::: warning
All classes passed to `class_names` must also be made discoverable using the C++ macro [`AUTO_APMS_BEHAVIOR_TREE_REGISTER_NODE`](https://autoapms.github.io/auto-apms/group__auto__apms__behavior__tree.html#ga5ce6f5e1249a2f980b0487ca8bb95c08).

This approach was inspired by [ROS 2 Composition](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Composition.html).
:::

| Argument | Required/Optional | Description |
| :--- | :---: | :--- |
| `target` | Required (Positional) | Shared library target implementing the behavior tree nodes given under `class_names`. |
| `class_names` | Required (Positional) | The unique, fully qualified names of behavior tree node classes (base class must be `BT::TreeNode`) being registered with this macro call and associated with the shared library target. |
| `NODE_MANIFEST` | Optional (Multi-Value-Keyword) | One or more relative paths (relative to `CMAKE_CURRENT_SOURCE_DIR`) or existing resource identities of node manifests. Multiple file paths are concatenated to a single one. |
| `NODE_MODEL_HEADER_TARGET` | Optional (Single-Value-Keyword) | Name of a single shared library target. If specified, generate a C++ header that defines model classes for all behavior tree nodes specified inside the node manifest files provided under `NODE_MANIFEST` and add it to the includes of the given target. |

### Example

```cmake
find_package(auto_apms_behavior_tree REQUIRED)
# ...
auto_apms_behavior_tree_register_nodes(my_target
  "my_namespace::MyCustomNodeFoo"
  "my_namespace::MyCustomNodeBar"
  NODE_MANIFEST
  "path/to/my_node_manifest.yaml"     # Relative file path
  "other_package::other_metadata_id"  # Resource identity
  NODE_MODEL_HEADER_TARGET
  other_target
)
```

::: info Learn more 🎓
Visit the tutorial [Implementing Custom Behavior Tree Nodes](../tutorial/implementing-behavior-tree-nodes.md) for more information.
:::

## `auto_apms_behavior_tree_register_behavior` {#register-behavior}

**Add a custom behavior definition to the workspace and register it with the resource index.**

This macro configures a unique "marker file" that contains essential information for building and executing behaviors at runtime. It uses `ament_index` for registering this resource with the workspace. Read [this guide](https://docs.ros.org/en/jazzy/How-To-Guides/Ament-CMake-Documentation.html#the-ament-index-explained) to learn more about the ament resource index.
<!-- markdown-link-check-disable-next-line -->
Use this macro for introducing custom behavior definitions tailored to your specific use case. If you simply want to add a behavior tree to the workspace, use [auto_apms_behavior_tree_register_trees](#register-trees) instead.

### Signature

```cmake
auto_apms_behavior_tree_register_behavior(<build_request>
  BUILD_HANDLER class_name
  [CATEGORY category]
  [ALIAS alias]
  [ENTRY_POINT entry_point]
  [NODE_MANIFEST node_manifest1 [node_manifest2 ...]]
  [MARK_AS_INTERNAL]
)
```

| Argument | Required/Optional | Description |
| :--- | :---: | :--- |
| `build_request` | Required (Positional) | Relative path to a file containing a behavior definition or a simple string. This argument determines the build request given to the behavior build handler provided with `BUILD_HANDLER`. **The user must make sure that the given build handler is able to interpret the given request**. |
| `BUILD_HANDLER` | Required | Fully qualified class name of the [behavior build handler](../concept/common-resources.md#behavior-build-handlers) that should be used by default to interpret the given behaviors. |
| `CATEGORY` | Optional (Single-Value-Keyword) | Category name to which the behaviors belong. If omitted, the default category is used. |
| `ALIAS` | Optional (Single-Value-Keyword) | Name for the behavior resource. If omitted, the file stem respectively the simple string is used as a behavior's alias. |
| `ENTRY_POINT` | Optional (Single-Value-Keyword) | Single point of entry for behavior execution. For behavior trees, this usually is the name of the root tree, but for other types of behaviors, this may be populated differently. |
| `NODE_MANIFEST` | Optional (Multi-Value-Keyword) | One or more relative paths (relative to `CMAKE_CURRENT_SOURCE_DIR`) or resource identities of existing node manifests. If specified, all associated behavior tree nodes can be loaded automatically thus are allowed to use in combination with the behavior defined by `build_request`. |
| `MARK_AS_INTERNAL` | Optional (Option) | If this option is set, the behavior is assigned a special category which indicates it is intended for internal use only. |

### Example

```cmake
find_package(auto_apms_behavior_tree REQUIRED)
# ...
auto_apms_behavior_tree_register_behavior(
  "path/to/my_custom_behavior_definition.yaml"
  BUILD_HANDLER "my_namespace::MyCustomBuildHandler"
)
```

## `auto_apms_behavior_tree_register_trees` {#register-trees}

**Make behavior trees available to the ROS 2 workspace by adding the corresponding behavior tree XML and node manifest YAML files to the resource index.**
<!-- markdown-link-check-disable-next-line -->
This is a specialization of [auto_apms_behavior_tree_register_behavior](#register-behavior) for behavior trees.
It provides a standardized way of registering behavior tree resources and allows the user to refer to them using a unique [resource identity](../concept/common-resources.md#tree-identity).

### Signature

```cmake
auto_apms_behavior_tree_register_trees(<paths>...
  [ALIAS_NAMESPACE alias_namespace]
  [NODE_MANIFEST node_manifest1 [node_manifest2 ...]]
  [MARK_AS_INTERNAL]
)
```

| Argument | Required/Optional | Description |
| :--- | :---: | :--- |
| `paths` | Required (Positional) | Behavior tree XML files to be added to this package's resources. The user must pass at least one. |
| `ALIAS_NAMESPACE` | Optional (Single-Value-Keyword) | Argument to customize the alias namespace used for all behavior trees to be registered. The default is to use the XML file stem as namspace. |
| `NODE_MANIFEST` | Optional (Multi-Value-Keyword) | One or more relative paths (relative to `CMAKE_CURRENT_SOURCE_DIR`) or resource identities of existing node manifests. If specified, all associated behavior tree nodes can be loaded automatically thus are allowed to use in combination with the trees specified under `paths`. |
| `MARK_AS_INTERNAL` | Optional (Option) | If this option is set, the behavior is assigned a special category which indicates it is intended for internal use only. |

### Example

```cmake
find_package(auto_apms_behavior_tree REQUIRED)
# ...
auto_apms_behavior_tree_register_trees(
  "path/to/my_behavior_tree.xml"
  "path/to/another_behavior_tree.xml"
  NODE_MANIFEST
  "path/to/my_node_manifest.yaml"     # Relative file path
  "other_package::other_metadata_id"  # Resource identity
)
```

::: info Learn more 🎓
Visit the tutorial [Building Behavior Trees: Graphical Approach](../tutorial/building-behavior-trees.md#graphical-approach) for more information.
:::

## `auto_apms_behavior_tree_register_build_handlers` {#register-build-handlers}

**Add behavior build handler plugins to the resource index.**

This macro must be called to make behavior build handlers available at runtime. They may be loaded using `TreeBuildHandlerLoader` (a subclass of `pluginlib::ClassLoader`).

### Signature

```cmake
auto_apms_behavior_tree_register_build_handlers(<target> <class_names>...)
```

::: warning
All classes passed to `class_names` must also be made discoverable using the C++ macro [`AUTO_APMS_BEHAVIOR_TREE_REGISTER_BUILD_HANDLER`](https://autoapms.github.io/auto-apms/group__auto__apms__behavior__tree.html#ga9e4ca44bb2265ef63f68487c01d32d92).
:::

| Argument | Required/Optional | Description |
| :--- | :---: | :--- |
| `target` | Required (Positional) | Shared library target implementing the behavior build handlers given under `class_names`. |
| `class_names` | Optional (Multi-Value-Keyword) | The unique, fully qualified names of classes inheriting from `TreeBuildHandler` being registered with this macro call and associated with the shared library target. |

### Example

```cmake
find_package(auto_apms_behavior_tree REQUIRED)
# ...
auto_apms_behavior_tree_register_build_handlers(my_target
  "my_namespace::MyCustomBuildHandlerFoo"
  "my_namespace::MyCustomBuildHandlerBar"
)
```

::: info Learn more 🎓
Visit the tutorial [Building Behavior Trees: Using `TreeBuildHandler`](../tutorial/building-behavior-trees.md#using-treebuildhandler) for more information.
:::

## `auto_apms_mission_register_missions` {#register-missions}

**Register mission configuration files with the resource index.**

This allows the user to refer to one of the registered mission configuration file using a [resource identity](../concept/common-resources.md#mission-config-identity)

### Signature

```cmake
auto_apms_mission_register_missions(<paths>...)
```

| Argument | Required/Optional | Description |
| :--- | :---: | :--- |
| `paths` | Required (Positional) | Mission configuration YAML files to be added to the resource index. The user must pass at least one. |

### Example

```cmake
find_package(auto_apms_mission REQUIRED)
# ...
auto_apms_mission_register_missions(
  "path/to/my_mission_config.yaml"
  "path/to/another_mission_config.yaml"
)
```

::: info Learn more 🎓
Visit the concept page about our [Mission Architecture](../concept/mission-architecture.md) for more information.
:::
