<!-- markdownlint-disable MD024 MD041 MD060 -->
| Registration Name | Class Name | Package |
| :--- | :---: | :---: |
| [ActivateController](#activatecontroller) | `auto_apms_ros2control::SwitchController` | auto_apms_ros2control |
| [CleanupController](#cleanupcontroller) | `auto_apms_ros2control::CleanupController` | auto_apms_ros2control |
| [ConfigureController](#configurecontroller) | `auto_apms_ros2control::ConfigureController` | auto_apms_ros2control |
| [DeactivateController](#deactivatecontroller) | `auto_apms_ros2control::SwitchController` | auto_apms_ros2control |
| [ListControllerTypes](#listcontrollertypes) | `auto_apms_ros2control::ListControllerTypes` | auto_apms_ros2control |
| [ListControllers](#listcontrollers) | `auto_apms_ros2control::ListControllers` | auto_apms_ros2control |
| [ListHardwareComponents](#listhardwarecomponents) | `auto_apms_ros2control::ListHardwareComponents` | auto_apms_ros2control |
| [ListHardwareInterfaces](#listhardwareinterfaces) | `auto_apms_ros2control::ListHardwareInterfaces` | auto_apms_ros2control |
| [LoadController](#loadcontroller) | `auto_apms_ros2control::LoadController` | auto_apms_ros2control |
| [ReloadControllerLibraries](#reloadcontrollerlibraries) | `auto_apms_ros2control::ReloadControllerLibraries` | auto_apms_ros2control |
| [SetHardwareComponentState](#sethardwarecomponentstate) | `auto_apms_ros2control::SetHardwareComponentState` | auto_apms_ros2control |
| [SwitchController](#switchcontroller) | `auto_apms_ros2control::SwitchController` | auto_apms_ros2control |
| [UnloadController](#unloadcontroller) | `auto_apms_ros2control::UnloadController` | auto_apms_ros2control |

## auto_apms_ros2control

### ActivateController

**Plugin Class:** `auto_apms_ros2control::SwitchController`

**C++ Model:** `auto_apms_ros2control::ActivateController`

**Node Type:** `Action`

**Description:** Activates a single ros2_control controller

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **controller** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` |  | Name of the controller to activate. |
| **strictness** | `std::string` | STRICT | Switching behavior on error. One of: BEST_EFFORT, STRICT, AUTO, FORCE_AUTO. |
| **controller_manager** | `std::string` | controller_manager | Name of the targeted controller manager. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **message** | `std::string` | ❌ | Service response: informational message. |

### CleanupController

**Plugin Class:** `auto_apms_ros2control::CleanupController`

**C++ Model:** `auto_apms_ros2control::CleanupController`

**Node Type:** `Action`

**Description:** Cleans up an inactive controller (transition back to the 'unconfigured' state)

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **controller** | `std::string` | ❌ | Name of the targeted controller. |
| **controller_manager** | `std::string` | controller_manager | Name of the targeted controller manager. |

### ConfigureController

**Plugin Class:** `auto_apms_ros2control::ConfigureController`

**C++ Model:** `auto_apms_ros2control::ConfigureController`

**Node Type:** `Action`

**Description:** Configures a loaded controller (transition to the 'inactive' state)

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **controller** | `std::string` | ❌ | Name of the targeted controller. |
| **controller_manager** | `std::string` | controller_manager | Name of the targeted controller manager. |

### DeactivateController

**Plugin Class:** `auto_apms_ros2control::SwitchController`

**C++ Model:** `auto_apms_ros2control::DeactivateController`

**Node Type:** `Action`

**Description:** Deactivates a single ros2_control controller

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **controller** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` |  | Name of the controller to deactivate. |
| **strictness** | `std::string` | STRICT | Switching behavior on error. One of: BEST_EFFORT, STRICT, AUTO, FORCE_AUTO. |
| **controller_manager** | `std::string` | controller_manager | Name of the targeted controller manager. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **message** | `std::string` | ❌ | Service response: informational message. |

### ListControllerTypes

**Plugin Class:** `auto_apms_ros2control::ListControllerTypes`

**C++ Model:** `auto_apms_ros2control::ListControllerTypes`

**Node Type:** `Action`

**Description:** Queries all controller types the controller manager is able to load

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **controller_manager** | `std::string` | controller_manager | Name of the targeted controller manager. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **base_classes** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Base classes of the available controller types (aligned with 'types'). |
| **types** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Names of all available controller types. |

### ListControllers

**Plugin Class:** `auto_apms_ros2control::ListControllers`

**C++ Model:** `auto_apms_ros2control::ListControllers`

**Node Type:** `Action`

**Description:** Queries the names and states of all controllers loaded by the controller manager

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **controller_manager** | `std::string` | controller_manager | Name of the targeted controller manager. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **inactive_controllers** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Names of all inactive controllers. |
| **active_controllers** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Names of all active controllers. |
| **controllers** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Names of all loaded controllers. |

### ListHardwareComponents

**Plugin Class:** `auto_apms_ros2control::ListHardwareComponents`

**C++ Model:** `auto_apms_ros2control::ListHardwareComponents`

**Node Type:** `Action`

**Description:** Queries the names and lifecycle states of all hardware components loaded by the controller manager

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **controller_manager** | `std::string` | controller_manager | Name of the targeted controller manager. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **inactive_components** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Names of all inactive hardware components. |
| **active_components** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Names of all active hardware components. |
| **components** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Names of all loaded hardware components. |

### ListHardwareInterfaces

**Plugin Class:** `auto_apms_ros2control::ListHardwareInterfaces`

**C++ Model:** `auto_apms_ros2control::ListHardwareInterfaces`

**Node Type:** `Action`

**Description:** Queries the names of all command and state interfaces registered with the controller manager

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **controller_manager** | `std::string` | controller_manager | Name of the targeted controller manager. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **claimed_command_interfaces** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Names of all currently claimed command interfaces. |
| **available_command_interfaces** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Names of all currently available command interfaces. |
| **state_interfaces** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Names of all state interfaces. |
| **command_interfaces** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` | ❌ | Names of all command interfaces. |

### LoadController

**Plugin Class:** `auto_apms_ros2control::LoadController`

**C++ Model:** `auto_apms_ros2control::LoadController`

**Node Type:** `Action`

**Description:** Loads a controller into the controller manager

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **controller** | `std::string` | ❌ | Name of the targeted controller. |
| **controller_manager** | `std::string` | controller_manager | Name of the targeted controller manager. |

### ReloadControllerLibraries

**Plugin Class:** `auto_apms_ros2control::ReloadControllerLibraries`

**C++ Model:** `auto_apms_ros2control::ReloadControllerLibraries`

**Node Type:** `Action`

**Description:** Reloads all available controller libraries of the controller manager

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **force_kill** | `bool` | false | Unload all loaded controllers before reloading the libraries. |
| **controller_manager** | `std::string` | controller_manager | Name of the targeted controller manager. |

### SetHardwareComponentState

**Plugin Class:** `auto_apms_ros2control::SetHardwareComponentState`

**C++ Model:** `auto_apms_ros2control::SetHardwareComponentState`

**Node Type:** `Action`

**Description:** Sets the lifecycle state of a hardware component loaded by the controller manager

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **target_state** | `std::string` | ❌ | Target lifecycle state. One of: unconfigured, inactive, active, finalized. |
| **component** | `std::string` | ❌ | Name of the targeted hardware component. |
| **controller_manager** | `std::string` | controller_manager | Name of the targeted controller manager. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **state** | `std::string` | ❌ | Label of the hardware component's resulting lifecycle state. |

### SwitchController

**Plugin Class:** `auto_apms_ros2control::SwitchController`

**C++ Model:** `auto_apms_ros2control::SwitchController`

**Node Type:** `Action`

**Description:** Activates and/or deactivates one or more ros2_control controllers in one controller manager update cycle

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **timeout** | `double` | 0.000000 | Timeout [s] before aborting pending controllers (0 = infinite). |
| **activate_asap** | `bool` | false | Activate the controllers as soon as their hardware dependencies are ready instead of waiting for all interfaces to become available. |
| **strictness** | `std::string` | STRICT | Switching behavior on error. One of: BEST_EFFORT, STRICT, AUTO, FORCE_AUTO. |
| **deactivate_controllers** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` |  | ';'-separated names of the controllers to deactivate. |
| **activate_controllers** | `std::vector<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >, std::allocator<std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > > >` |  | ';'-separated names of the controllers to activate. |
| **controller_manager** | `std::string` | controller_manager | Name of the targeted controller manager. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **message** | `std::string` | ❌ | Service response: informational message. |

### UnloadController

**Plugin Class:** `auto_apms_ros2control::UnloadController`

**C++ Model:** `auto_apms_ros2control::UnloadController`

**Node Type:** `Action`

**Description:** Unloads a controller from the controller manager

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **controller** | `std::string` | ❌ | Name of the targeted controller. |
| **controller_manager** | `std::string` | controller_manager | Name of the targeted controller manager. |
