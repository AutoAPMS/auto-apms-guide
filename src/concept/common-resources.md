---
order: 40
---
# Common Resources
<!-- markdownlint-disable MD024 -->  

This page provides the most relevant information for all behavior related resources required when using AutoAPMS. To manage these resources, we incorporate one of ROS's most important core packages: [`ament_index`](https://github.com/ament/ament_index?tab=readme-ov-file). It offers a concept for distributing non-compiled files within a ROS 2 workspace. It's a powerful tool that we heavily rely on to automate troublesome filesystem related processes.

*High-level:*

- [Behavior Trees](#behavior-trees)
- [Mission Configurations](#mission-configurations)

*Low-level:*

- [Behavior Build Handlers](#behavior-build-handlers)
- [Behavior Tree Node Manifests](#behavior-tree-node-manifests)
- [Behavior Tree Node Models](#behavior-tree-node-models)

## Behavior Trees

Probably the most important non-compiled resource when using AutoAPMS is the behavior tree XML file. The [XML Schema](https://www.behaviortree.dev/docs/learn-the-basics/xml_format) looks similar to this:

```xml
<root BTCPP_format="4" main_tree_to_execute="RootTree">
  <!-- Each TreeDocument may have zero or more behavior tree elements -->
  <BehaviorTree ID="RootTree">
    <!-- Each behavior tree element has exactly one child. This child may again have zero or more children -->
  </BehaviorTree>
  <BehaviorTree ID="AnotherTree">
    <!-- ... -->
  </BehaviorTree>
</root>
```

Behavior tree resources are created using [`auto_apms_behavior_tree_register_trees`](../reference/cmake.md#register-trees).

::: info Learn more 🎓
Visit the tutorial [Building Behavior Trees: Graphical Approach](../tutorial/building-behavior-trees.md#graphical-approach) for more information about how behavior tree resources are created.
:::

### Resource Identity Format {#tree-identity}

This is the full signature of a behavior tree's resource identity:

- `<package_name>::<tree_file_stem>::<tree_name>`

| Token Name | Description |
| :---: | :--- |
| `<package_name>` | Name of the ROS 2 package that registers the resource. |
| `<tree_file_stem>` | Name of the behavior tree XML file (without extension) that contains the data. |
| `<tree_name>` | Name of a specific tree inside `<tree_file_stem>.xml` that is supposed to be the root tree. |

#### Example

Given the following configuration of a package called `my_package`:

```cmake [CMakeLists.txt]
project(my_package)
# ...
auto_apms_behavior_tree_register_trees(
  "behavior/my_behavior_tree.xml"
  "behavior/another_behavior_tree.xml"
  # ...
)
```

The available node manifest resources can be queried like this:

```cpp [source.cpp]
#include "auto_apms_behavior_tree_core/tree/tree_resource.hpp"
#include "auto_apms_behavior_tree_core/tree/tree_document.hpp"

using TreeResource = auto_apms_behavior_tree::core::TreeResource;
using TreeResourceIdentity = auto_apms_behavior_tree::core::TreeResourceIdentity;
using TreeDocument = auto_apms_behavior_tree::core::TreeDocument;
 
// For example, use the fully qualified tree resource identity signature
const std::string identity_string = "my_package::my_behavior_tree::MyTreeName";
 
// You may use the proxy class for a tree resource identity
TreeResourceIdentity identity(identity_string);
TreeResource resource(identity);
 
// Or instantiate the resource object directly using the corresponding identity
TreeResource resource(identity_string);
 
// The resource object may for example be used with TreeDocument
TreeDocument doc;
doc.mergeResource(resource);  // Add "MyTreeName" to the document
 
// This also works, so creating a resource object is not strictly necessary
doc.mergeResource(identity_string)
 
// The simplest approach is this
doc.mergeResource("my_package::my_behavior_tree::MyTreeName");
```

## Mission Configurations

```yaml [mission_config.yaml]
# Executed once on startup
BRINGUP:
  - <tree_resource_identity>
  # ...

# Nominal mission behaviors executed subsequently
MISSION:
  - <tree_resource_identity>
  # ...

# Contingency behaviors. Mission will be resumed after monitor returns FAILURE
CONTINGENCY:
  <tree_resource_identity>: <tree_resource_identity>  # Monitor_ID: Handler_ID
  # ...

# Emergency behaviors. Mission will be aborted when finished
EMERGENCY:
  <tree_resource_identity>: <tree_resource_identity>
  # ...

# Executed after nominal mission and potentially emergency handlers
SHUTDOWN:
  - <tree_resource_identity>
  # ...
```

All of the shown keys except `MISSION` are optional.

::: info Learn more 🎓
Visit the concept page about our [Mission Architecture](./mission-architecture.md) for more information about how to configure missions.
:::

### Resource Identity Format {#mission-config-identity}

This is the full signature of a mission configuration's resource identity:

- `<package_name>::<config_file_stem>`

| Token Name | Description |
| :---: | :--- |
| `<package_name>` | Name of the ROS 2 package that registers the resource. |
| `<config_file_stem>` | Name of the mission configuration YAML file (without extension) that contains the data. |

## Behavior Build Handlers

Behavior build handlers allow to integrate customized algorithms for creating behavior trees with AutoAPMS's [behavior executor](./behavior-executor.md). They are used to define rules for answering the "build requests" formulated when [deploying behaviors](../tutorial/deploying-behaviors.md).

AutoAPMS provides the following commonly used build handlers out of the box:

- `auto_apms_behavior_tree::TreeFromResourceBuildHandler`

    **Loads a specific behavior tree from the workspace's resource index.**

    This build handler parses the installed [behavior tree resources](#behavior-trees) and builds one of the trees that have previously been registered by the user. The **build request is expected to be the resource identity** of the desired behavior tree.

- `auto_apms_behavior_tree::TreeFromStringBuildHandler`

    **Creates the tree by parsing the provided XML string.**

    The user must encode the behavior tree to be created using the [XML format](https://www.behaviortree.dev/docs/learn-the-basics/xml_format). The **build request is expected to be the XML string**. This build handler additionally requires the user to specify a node manifest for all the nodes used inside the tree.

::: info Learn more 🎓
Visit the tutorial [Building Behavior Trees: Using `TreeBuildHandler`](../tutorial/building-behavior-trees.md#using-treebuildhandler) to learn how to implement your own abstractions.
:::

### Resource Identity Format {#build-handler-identity}

This is the full signature of a build handler's resource identity:

- `<namespace>::<class_name>`

This is also referred to as the "fully qualified class name". We use the same approach for referring to specific build handler plugin as [ROS 2 Composition](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Composition.html) does for `rclcpp::Node` components.

| Token Name | Description |
| :---: | :--- |
| `<namespace>` | Full C++ namespace that the class `<class_name>` can be found in. It can be flat `foo` or arbitrarily nested `foo::bar` (individual names must be separated by `::`). |
| `<class_name>` | Name you specified when registering the build handler class using the `class` keyword in C++. |

## Behavior Tree Node Manifests

Behavior tree node manifests are YAML files used for specifying the [`NodeRegistrationOptions`](https://autoapms.github.io/auto-apms/structauto__apms__behavior__tree_1_1core_1_1NodeRegistrationOptions.html) required when loading node plugins implemented by the user. Programmatically, a node manifest is represented by [`NodeManifest`](https://autoapms.github.io/auto-apms/classauto__apms__behavior__tree_1_1core_1_1NodeManifest.html). The format of the corresponding YAML file must be as follows:

```yaml [node_manifest.yaml]
MyCustomNodeRegistrationName:  # Registration name of the node
  class_name: my_namespace::MyCustomNodeClass
  description: "My custom description."
  topic: (input:topic)
  port_alias: {}  # Must be a map
  port_default: {}  # Must be a map
  hidden_ports: []  # Must be a list
  wait_timeout: 3
  request_timeout: 2
  allow_unreachable: false
  logger_level: INFO
  extra: ~  # Can be any valid yaml object

AnotherCoolNodeName:
  parent: other_namespace::OtherClass.OtherRegistrationName

# ...
```

| Parameter Name | Required/Optional | Interpreted Type | Description |
| :--- | :---: | :---: | :--- |
| `parent` | Optional | `std::string` | Identity string formatted like `<package_name>::<manifest_alias>.<node_name>` referencing another node in an upstream node manifest. The node will inherit all registration options from its parent unless they are overridden by explicitly specifying them. This allows for avoiding redundancy when multiple nodes share most of their registration options. |
| `class_name` | Required (if `parent` is omitted) | `std::string` | Fully qualified class name (includes all namespace levels separated by `::`) of the behavior tree node plugin that implements the functionality accessible to the behavior tree using the given registration name. We use the same approach for referring to specific behavior tree nodes as [ROS 2 Composition](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Composition.html) does for `rclcpp::Node` components. |
| `description` | Optional | `std::string` | Short description of the behavior tree node's purpose and use-case. |
| `topic` | Optional | `std::string` | *Only relevant for ROS 2 interface nodes.* Name of the ROS 2 action/service/topic to connect with. It is possible to use a port's value to define this parameter at runtime by using the special pattern `(input:<port_name>)` and replacing `<port_name>` with the desired input port name. |
| `port_alias` | Optional | `std::map<std::string, std::string>` | Provides the possibility to rename ports implemented by `class_name`. This is useful when a node implementation is used in a different context and the meaning of some of the ports has changed. In this case, it's possible to define a more descriptive port name. The description can also be updated by appending it within round brackets (e.g. `original name: alias_name (new description)`). |
| `port_default` | Optional | `std::map<std::string, std::string>` | Provides the possibility to define custom default values for the ports implemented by `class_name`. This will override the "hard-coded" value and allows for configuring a behavior tree node without touching its source file. |
| `hidden_ports` | Optional | `std::vector<std::string>` | List of port names to hide in the node model for visualization tools like Groot2. This will not change the implementation, but only modify the generated node model. |
| `wait_timeout` | Optional | `double` | *Only relevant for ROS 2 interface nodes.* Period [s] (measured from tree construction) after the server is considered unreachable. |
| `request_timeout` | Optional | `double` | *Only relevant for ROS 2 interface nodes.* Period [s] (measured from sending a goal request) after the node aborts waiting for a server response. |
| `allow_unreachable` | Optional | `bool` | *Only relevant for ROS 2 interface nodes.* Flag whether to tolerate if the action/service is unreachable when trying to create the client. If set to `true`, a warning is logged. Otherwise, an exception is raised. |
| `logger_level` | Optional | `std::string` | *Only relevant for ROS 2 interface nodes.* Minimum severity level enabled for logging using the ROS 2 Logger API. |
| `extra` | Optional | `YAML::Node` | Flexible YAML node which allows providing additional and customized registration options to the behavior tree node implementation. May contain any arbitrary YAML structure. |

You should understand that registration options are directly associated with a particular registration name.

You may include an arbitrary number of nodes in a node manifest. It's also possible to use the same implementation (specify the same node class name) for different registration names. However, the node registration names within a node manifest must be unique.

Only the `class_name` option is required. However, it can be inherited by populating `parent`. The example above shows the default values that will be used if any optional parameters are omitted. The node classes referred to by `class_name` must have been registered using [`auto_apms_behavior_tree_register_nodes`](../reference/cmake.md#auto-apms-behavior-tree-register-nodes) before the node manifest is parsed (specifying the node manifest in the same macro call is ok).

::: info Learn more 🎓
Visit the tutorial [Implementing Behavior Tree Nodes: Adding Node Manifests](../tutorial/implementing-behavior-tree-nodes.md#adding-node-manifests) for more information about how to register a node manifest with the resource index.
:::

### Resource Identity Format {#node-manifest-identity}

This is the full signature of a node manifest's resource identity:

- `<package_name>::<manifest_alias>`

| Token Name | Description |
| :---: | :--- |
| `<package_name>` | Name of the ROS 2 package that registers the resource. |
| `<manifest_alias>` | Alias for the behavior tree node manifest that is automatically generated at configuration time when specifying the `NODE_MANIFEST` keyword argument of [`auto_apms_behavior_tree_register_nodes`](../reference/cmake.md#register-nodes). |

::: tip Convention 📜
Typically, `auto_apms_behavior_tree_register_nodes` is called only once per package and the shared library target linking against all behavior tree node source files should be named `behavior_tree_nodes`. So, by convention, the identity of a package's node manifest is `<package_name>::behavior_tree_nodes`.
:::

#### Example

Given the following configuration of a package called `my_package`:

```cmake [CMakeLists.txt]
project(my_package)
# ...
auto_apms_behavior_tree_register_nodes(behavior_tree_nodes
  # ...
  NODE_MANIFEST
  "config/my_node_manifest.yaml"
)
# ...
auto_apms_behavior_tree_register_trees(
  "behavior/my_behavior_tree.xml"
  NODE_MANIFEST
  "config/my_node_manifest.yaml"
)
```

The available node manifest resources can be queried like this:

::: code-group

```cmake [CMakeLists.txt]
auto_apms_behavior_tree_register_nodes(another_target
  # ...
  NODE_MANIFEST
  "my_package::behavior_tree_nodes"
  "my_package::my_behavior_tree"
)
# OR
auto_apms_behavior_tree_register_trees(
  # ...
  NODE_MANIFEST
  "my_package::behavior_tree_nodes"
  "my_package::my_behavior_tree"
)
```

```cpp [source.cpp]
#include "auto_apms_behavior_tree_core/node/node_manifest.hpp"

using namespace auto_apms_behavior_tree::core;

NodeManifest manifest1 = NodeManifest::fromResourceIdentity("my_package::behavior_tree_nodes");
NodeManifest manifest2 = NodeManifest::fromResourceIdentity("my_package::my_behavior_tree");
```

:::

## Behavior Tree Node Models

Behavior tree node models are represented in two ways:

- By a XML file usually created with `BT::writeTreeNodesModelXML`
- By C++ classes representing individual nodes defined inside a `.hpp` file

Those resources are automatically generated when the `NODE_MANIFEST` keyword argument of [`auto_apms_behavior_tree_register_nodes`](../reference/cmake.md#register-nodes) or [`auto_apms_behavior_tree_register_trees`](../reference/cmake.md#register-trees) is given.

### Node Model XML File

Typically used when [building behavior trees graphically](../tutorial/building-behavior-trees.md#graphical-approach).

The XML file holds metadata for multiple nodes. This data enables to determine the type of each node as well as the names and types of the associated data ports. After the ROS 2 package was installed, the file can be found under

::: tabs

=== Normal Install

`install`/`<my_package>`/`share`/`<my_package>`/`auto_apms`/`auto_apms_behavior_tree_core`/`metadata`

=== Merge Install

`install`/`share`/`<my_package>`/`auto_apms`/`auto_apms_behavior_tree_core`/`metadata`

:::

::: info You can also inspect the build directory
When building with `symlink-install` enabled, the install directory only contains symbolic links to the node model files. The originally generated files reside in

`build`/`<my_package>`/`auto_apms_behavior_tree_core`
:::

The XML schema looks something like this:

```xml
<root BTCPP_format="4">
    <TreeNodesModel>
        <NodeType ID="NodeRegistrationName">
            <input_port name="port_name" type="std::string">Port description.</input_port>
            <output_port name="another_port_name" type="bool">Port description.</input_port>
        </NodeType>
        <!-- ... -->
    </TreeNodesModel>
</root>
```

### C++ Node Model Header

Typically used when [building behavior trees programmatically](../tutorial/building-behavior-trees.md#programmatic-approach).

The C++ node models can be passed as template arguments to specific methods of `TreeDocument`. They implement an API that complies with the node's entry in the node model XML file.

::: info Learn more 🎓
Check out the usage example [Using `TreeDocument`: Inserting custom nodes](../tutorial/building-behavior-trees.md#inserting-custom-nodes) for more information about how node models are created and may be used in a C++ source file.
:::
