---
order: 0
sidebar: ros2behavior
---
# `ros2behavior` CLI Reference

We offer a powerful extension to the ROS 2 command line interface (CLI) by introducing the `ros2 behavior` verb, an ideal supplement to the functionality described by this user guide. It introduces convenient tab completion and allows for quick introspection of installed AutoAPMS resources. It is the recommended way of interacting with everything behavior-related in the workspace.

To install this extension, make sure to build the `auto_apms_ros2behavior` package and source the workspace. Afterwards, you may run

```bash
ros2 behavior -h
```

## **`ros2 behavior`**

This is the entry point for all subcommands.

**Usage:**

```bash
ros2 behavior <subcommand> [arguments] [options]
```

**Subcommands:**

- [`list`](#ros2-behavior-list) - List all available behavior resources
- [`show`](#ros2-behavior-show) - Show the content of a behavior resource
- [`run`](#ros2-behavior-run) - Execute a behavior locally
- [`send`](#ros2-behavior-send) - Send a behavior to a running executor
- [`node`](#ros2-behavior-node) - Subcommand for behavior tree node operations

### **`ros2 behavior list`**

List all available behavior resources in the system.

**Usage:**

```bash
ros2 behavior list [options]
```

**Options:**

| Option | Description |
|---|---|
| `-c, --categories [CATEGORIES ...]` | List behavior resources in specified categories. If no category is given, all resources are listed. |
| `--include-internal` | Include behaviors marked as internal (flag). |

**Examples:**

```bash
# List all available behaviors
ros2 behavior list

# List behaviors in specific categories
ros2 behavior list -c tree mission
```

**Output:**
Behaviors are grouped by category and displayed hierarchically:

```text
mission/
  - navigation::explore
tree/
  - navigation::goto_waypoint::Main
  - navigation::follow_path::Main
```

---

### **`ros2 behavior show`**

Display the content/build request of a specific behavior resource.

**Usage:**

```bash
ros2 behavior show <behavior>
```

**Arguments:**

- `behavior` - Identity string of the behavior resource to show

**Examples:**

```bash
# Show a specific behavior
ros2 behavior show navigation::goto_waypoint::Main
```

**Output:**
Displays the build request content for the specified behavior resource.

---

### **`ros2 behavior run`**

Execute a behavior tree locally in a standalone executor.

**Usage:**

```bash
ros2 behavior run [behavior] [options]
```

**Arguments:**

- `behavior` - Identity string of the behavior resource to execute (optional)

**Options:**

| Option | Description |
|---|---|
| `--build-request <string>` | Build request to pass to the build handler (overrides build request associated with `behavior` if given). |
| `--build-handler <namespace>::<class_name>` | Build handler to load (overrides build handler associated with `behavior` if given). |
| `--entry-point <string>` | Entry point to pass to the build handler (overrides entry point associated with `behavior` if given). |
| `--node-manifest <string>` | Node manifest resource to pass to the build handler (overrides node manifest associated with `behavior` if given). |
| `--blackboard [key:=value ...]` | Blackboard variables to pass to the behavior tree. |
| `--tick-rate <float>` | Tick rate for the behavior tree in seconds. |
| `--groot2-port <int>` | Port for Groot2 visualization (disabled by default). |
| `--state-change-logger` | Enable the state change logger (flag). |
| `--logging <level>` | Set logger level (debug, info, warn, error, fatal). |

**Examples:**

```bash
# Run a behavior with default settings
ros2 behavior run navigation::explore

# Run with custom blackboard variables
ros2 behavior run navigation::explore --blackboard target_x:=10.0 target_y:=5.0

# Run a dynamic request without predefined behavior
ros2 behavior run --build-request "custom_request" --build-handler custom_namespace::CustomBuildHandler
```

---

### **`ros2 behavior send`**

Send a behavior tree to a running executor and start execution.

**Usage:**

```bash
ros2 behavior send <executor_name> [behavior] [options]
```

**Arguments:**

- `executor_name` - Name of the behavior tree executor to send the tree to
- `behavior` - Identity string of the behavior resource to execute

**Options:**

| Option | Description |
|---|---|
| `--build-request <string>` | Build request to pass to the build handler (overrides build request associated with `behavior` if given). |
| `--build-handler <namespace>::<class_name>` | Build handler to load (overrides build handler associated with `behavior` if given). |
| `--entry-point <string>` | Entry point to pass to the build handler (overrides entry point associated with `behavior` if given). |
| `--node-manifest <string>` | Node manifest resource to pass to the build handler (overrides node manifest associated with `behavior` if given). |
| `--blackboard [key:=value ...]` | Blackboard variables to pass to the behavior tree. |
| `--keep-blackboard` | Do not explicitly clean the blackboard before execution (flag). |
| `--tick-rate <float>` | Tick rate for the behavior tree in seconds (keeps current if omitted). |
| `--groot2-port <int>` | Port for Groot2 visualization (keeps current if omitted). |
| `--state-change-logger <true/false>` | Enable/disable state change logger (keeps current if omitted). |
| `--logging <level>` | Set logger level (keeps current if omitted). |

**Examples:**

```bash
# Send behavior to executor
ros2 behavior send my_executor navigation::explore

# Send with blackboard variables
ros2 behavior send my_executor navigation::explore --blackboard target_x:=10.0 target_y:=5.0

# Send a dynamic request without predefined behavior
ros2 behavior send my_executor --build-request "custom_request" --build-handler custom_namespace::CustomBuildHandler
```

---

## **`ros2 behavior node`**

Subcommand for behavior tree node-related operations. Contains several subcommands itself.

**Usage:**

```bash
ros2 behavior node <subcommand> [options]
```

**Subcommands:**

- [`plugins`](#ros2-behavior-node-plugins) - List all available behavior tree node plugins
- [`manifest`](#ros2-behavior-node-manifest) - Inspect registered behavior tree node manifests
- [`model`](#ros2-behavior-node-model) - Inspect behavior tree node models
- [`call`](#ros2-behavior-node-call) - Execute a single behavior tree node locally

### **`ros2 behavior node plugins`**

List all available behavior tree node plugins grouped by package.

**Usage:**

```bash
ros2 behavior node plugins
```

**Examples:**

```bash
ros2 behavior node plugins
```

**Output:**

```text
Package: auto_apms_behavior_tree
  - ActionNode
  - ConditionNode
  - DecoratorNode
Package: my_custom_package
  - CustomActionNode
  - CustomConditionNode
```

### **`ros2 behavior node manifest`**

Inspect registered behavior tree node manifests and their registered nodes.

**Usage:**

```bash
ros2 behavior node manifest [identity] [node_name]
```

**Arguments:**

- `identity` - Identity string of a node manifest to inspect (optional)
- `node_name` - Registration name for a specific node from the manifest (optional)

**Examples:**

```bash
# List all manifests and their nodes
ros2 behavior node manifest

# Show nodes in a specific manifest
ros2 behavior node manifest navigation::behavior_tree_nodes

# Show details for a specific node
ros2 behavior node manifest navigation::behavior_tree_nodes GeneratePath
```

**Output:**

- Without arguments: Lists all manifests and their registered nodes
- With identity: Shows all nodes in that manifest
- With both: Shows YAML configuration for the specific node

### **`ros2 behavior node model`**

Inspect behavior tree node models, showing node types, ports, and detailed information.

**Usage:**

```bash
ros2 behavior node model <manifest> [node_name]
```

**Arguments:**

- `manifest` - Identity string of a node manifest
- `node_name` - Registration name for a specific node (optional)

**Examples:**

```bash
# Show overview of all nodes in a manifest
ros2 behavior node model navigation::behavior_tree_nodes

# Show detailed information for a specific node
ros2 behavior node model navigation::behavior_tree_nodes GeneratePath
```

**Output:**

- Without node_name: Shows grouped overview of all nodes by type
- With node_name: Shows detailed information including ports, descriptions, and defaults

### **`ros2 behavior node call`**

Execute a single behavior tree node locally.

**Usage:**

```bash
ros2 behavior node call <manifest> <node_name> [port_values...] [options]
```

**Arguments:**

- `manifest` - Identity string of the node manifest
- `node_name` - Registration name of the node to call
- `port_values` - Port values in `port:=value` format

**Options:**

| Option | Description |
|---|---|
| `--logging <level>` | Set logger level for the executor node. |

**Examples:**

```bash
# Call a node with port values
ros2 behavior node call navigation::behavior_tree_nodes GeneratePath target_x:=10.0 target_y:=5.0

# Call with debug logging
ros2 behavior node call navigation::behavior_tree_nodes GeneratePath target_x:=10.0 --logging debug
```
