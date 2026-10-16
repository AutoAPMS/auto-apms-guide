<!-- markdownlint-disable MD024 MD041 MD060 -->
| Registration Name | Class Name | Package |
| :--- | :---: | :---: |
| [CheckArmingState](#checkarmingstate) | `auto_apms_px4::CheckArmingState` | auto_apms_px4 |
| [CheckModeCompleted](#checkmodecompleted) | `auto_apms_px4::CheckModeCompleted` | auto_apms_px4 |
| [CheckNavState](#checknavstate) | `auto_apms_px4::CheckNavState` | auto_apms_px4 |
| [GetGlobalPosition](#getglobalposition) | `auto_apms_px4::GetPosition<GlobalPositionMsg>` | auto_apms_px4 |
| [GetLocalPosition](#getlocalposition) | `auto_apms_px4::GetPosition<LocalPositionMsg>` | auto_apms_px4 |
| [GetModeNavState](#getmodenavstate) | `auto_apms_px4::GetModeNavState` | auto_apms_px4 |
| [GoTo](#goto) | `auto_apms_px4::GoToAction` | auto_apms_px4 |
| [GoToGlobal](#gotoglobal) | `auto_apms_px4::GoToAction` | auto_apms_px4 |
| [GoToLocal](#gotolocal) | `auto_apms_px4::GoToAction` | auto_apms_px4 |
| [OffsetGlobalPosition](#offsetglobalposition) | `auto_apms_px4::OffsetGlobalPosition` | auto_apms_px4 |
| [SendCmdDoReposition](#sendcmddoreposition) | `auto_apms_px4::SendCmdDoReposition` | auto_apms_px4 |
| [SendCmdSetNavState](#sendcmdsetnavstate) | `auto_apms_px4::SendCmdSetNavState` | auto_apms_px4 |
| [SendVehicleCommand](#sendvehiclecommand) | `auto_apms_px4::SendVehicleCommand` | auto_apms_px4 |
| [SetActuators](#setactuators) | `auto_apms_px4::SetActuatorsAction` | auto_apms_px4 |

## auto_apms_px4

### CheckArmingState

**Plugin Class:** `auto_apms_px4::CheckArmingState`

**C++ Model:** `auto_apms_px4::CheckArmingState`

**Node Type:** `Condition`

**Description:** Succeeds once the vehicle's PX4 arming state matches the expected one (1 = disarmed, 2 = armed)

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **arming_state** | `int` | ❌ | Expected PX4 arming state the vehicle should be in (1 = disarmed, 2 = armed). |

### CheckModeCompleted

**Plugin Class:** `auto_apms_px4::CheckModeCompleted`

**C++ Model:** `auto_apms_px4::CheckModeCompleted`

**Node Type:** `Condition`

**Description:** Succeeds once the PX4 mode with the expected nav_state reports successful completion (mode_completed)

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **nav_state** | `int` | ❌ | PX4 navigation state (mode id) whose completion is awaited. |

### CheckNavState

**Plugin Class:** `auto_apms_px4::CheckNavState`

**C++ Model:** `auto_apms_px4::CheckNavState`

**Node Type:** `Condition`

**Description:** Succeeds once the vehicle's active PX4 navigation state (nav_state) matches the expected one

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **nav_state** | `int` | ❌ | Expected PX4 navigation state (mode id) the vehicle should be in. |

### GetGlobalPosition

**Plugin Class:** `auto_apms_px4::GetPosition<GlobalPositionMsg>`

**C++ Model:** `auto_apms_px4::GetGlobalPosition`

**Node Type:** `Condition`

**Description:** Gets the current position of the vehicle in the global coordinate frame

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **alt** | `double` | {alt} | Current altitude in meter (AMSL) |
| **lon** | `double` | {lon} | Current longitude in degree [°] |
| **lat** | `double` | {lat} | Current latitude in degree [°] |
| **vector** | `Eigen::Matrix<double, -1, -1, 0, -1, -1>` | {pos_vec} | Current global position vector (latitude [°], longitude [°], altitude AMSL [m]) |

### GetLocalPosition

**Plugin Class:** `auto_apms_px4::GetPosition<LocalPositionMsg>`

**C++ Model:** `auto_apms_px4::GetLocalPosition`

**Node Type:** `Condition`

**Description:** Gets the current position of the vehicle in the local coordinate frame

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **down** | `double` | {down} | Current down [m] relative to origin |
| **east** | `double` | {east} | Current east [m] relative to origin |
| **north** | `double` | {north} | Current north [m] relative to origin |
| **vector** | `Eigen::Matrix<double, -1, -1, 0, -1, -1>` | {pos_vec} | Current local position vector (north [m], east [m], down [m]) |

### GetModeNavState

**Plugin Class:** `auto_apms_px4::GetModeNavState`

**C++ Model:** `auto_apms_px4::GetModeNavState`

**Node Type:** `Condition`

**Description:** Resolves a PX4 mode's name to its nav_state (mode id) - builtin modes plus any registered custom mode

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **mode_name** | `std::string` | ❌ | Name of the registered PX4 mode to resolve. |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **nav_state** | `int` | ❌ | Resolved PX4 navigation state (mode id) of the named mode. |

### GoTo

**Plugin Class:** `auto_apms_px4::GoToAction`

**C++ Model:** `auto_apms_px4::GoTo`

**Node Type:** `Action`

**Description:** Commands the vehicle to go to a specified position. The reference frame can be set via the 'frame' input port

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **reached_thresh_yaw** | `float` | 7.000000 | Maximum heading error [°] under which the position is considered reached |
| **reached_thresh_vel** | `float` | 0.300000 | Maximum velocity error [m/s] under which the position is considered reached |
| **reached_thresh_pos** | `float` | 0.500000 | Maximum position error [m] under which the position is considered reached |
| **max_heading_rate** | `float` | 30.000000 | Maximum heading rate [°/s] |
| **max_vertical_vel** | `float` | 5.000000 | Maximum vertical velocity [m/s] |
| **max_horizontal_vel** | `float` | 10.000000 | Maximum horizontal velocity [m/s] |
| **yaw** | `float` | ❌ | Desired yaw position in degree from north (heading) [-180°, 180) |
| **z** | `float` | ❌ | Override vector entry Z |
| **y** | `float` | ❌ | Override vector entry Y |
| **x** | `float` | ❌ | Override vector entry X |
| **vector** | `Eigen::Matrix<double, -1, -1, 0, -1, -1>` | ❌ | Target position as a row vector (separated by ';') |
| **frame** | `std::string` | global | Reference frame: 'global' (Latitude, longitude, altitude (AMSL)) or 'local' (North, east, down from start) |

### GoToGlobal

**Plugin Class:** `auto_apms_px4::GoToAction`

**C++ Model:** `auto_apms_px4::GoToGlobal`

**Node Type:** `Action`

**Description:** Commands the vehicle to go to a specified position in the global coordinate frame

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **reached_thresh_yaw** | `float` | 7.000000 | Maximum heading error [°] under which the position is considered reached |
| **reached_thresh_vel** | `float` | 0.300000 | Maximum velocity error [m/s] under which the position is considered reached |
| **reached_thresh_pos** | `float` | 0.500000 | Maximum position error [m] under which the position is considered reached |
| **max_heading_rate** | `float` | 30.000000 | Maximum heading rate [°/s] |
| **max_vertical_vel** | `float` | 5.000000 | Maximum vertical velocity [m/s] |
| **max_horizontal_vel** | `float` | 10.000000 | Maximum horizontal velocity [m/s] |
| **yaw** | `float` | ❌ | Desired yaw position in degree from north (heading) [-180°, 180) |
| **z** | `float` | ❌ | Override vector entry Z |
| **y** | `float` | ❌ | Override vector entry Y |
| **x** | `float` | ❌ | Override vector entry X |
| **vector** | `Eigen::Matrix<double, -1, -1, 0, -1, -1>` | ❌ | Target position as a row vector (separated by ';') |
| **frame** | `std::string` | global | Reference frame: 'global' (Latitude, longitude, altitude (AMSL)) or 'local' (North, east, down from start) |

### GoToLocal

**Plugin Class:** `auto_apms_px4::GoToAction`

**C++ Model:** `auto_apms_px4::GoToLocal`

**Node Type:** `Action`

**Description:** Commands the vehicle to go to a specified position in the local coordinate frame

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **reached_thresh_yaw** | `float` | 7.000000 | Maximum heading error [°] under which the position is considered reached |
| **reached_thresh_vel** | `float` | 0.300000 | Maximum velocity error [m/s] under which the position is considered reached |
| **reached_thresh_pos** | `float` | 0.500000 | Maximum position error [m] under which the position is considered reached |
| **max_heading_rate** | `float` | 30.000000 | Maximum heading rate [°/s] |
| **max_vertical_vel** | `float` | 5.000000 | Maximum vertical velocity [m/s] |
| **max_horizontal_vel** | `float` | 10.000000 | Maximum horizontal velocity [m/s] |
| **yaw** | `float` | ❌ | Desired yaw position in degree from north (heading) [-180°, 180) |
| **z** | `float` | ❌ | Override vector entry Z |
| **y** | `float` | ❌ | Override vector entry Y |
| **x** | `float` | ❌ | Override vector entry X |
| **vector** | `Eigen::Matrix<double, -1, -1, 0, -1, -1>` | ❌ | Target position as a row vector (separated by ';') |
| **frame** | `std::string` | local | Reference frame: 'global' (Latitude, longitude, altitude (AMSL)) or 'local' (North, east, down from start) |

### OffsetGlobalPosition

**Plugin Class:** `auto_apms_px4::OffsetGlobalPosition`

**C++ Model:** `auto_apms_px4::OffsetGlobalPosition`

**Node Type:** `Action`

**Description:** Offsets a global position (lat, lon, alt) by a local NED vector [m] - e.g. to compute targets for SendCmdDoReposition

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **north** | `double` | 0.000000 | Offset towards north [m] |
| **east** | `double` | 0.000000 | Offset towards east [m] |
| **alt** | `double` | {alt} | Altitude [m] (AMSL) of the position to offset |
| **down** | `double` | 0.000000 | Offset downwards [m] (negative to go up) |
| **lon** | `double` | {lon} | Longitude [°] of the position to offset |
| **lat** | `double` | {lat} | Latitude [°] of the position to offset |

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **target_vector** | `Eigen::Matrix<double, -1, -1, 0, -1, -1>` | {target_vec} | Resulting global position vector (latitude [°], longitude [°], altitude AMSL [m]) |
| **target_alt** | `double` | {target_alt} | Resulting altitude [m] (AMSL) |
| **target_lon** | `double` | {target_lon} | Resulting longitude [°] |
| **target_lat** | `double` | {target_lat} | Resulting latitude [°] |

### SendCmdDoReposition

**Plugin Class:** `auto_apms_px4::SendCmdDoReposition`

**C++ Model:** `auto_apms_px4::SendCmdDoReposition`

**Node Type:** `Action`

**Description:** Commands the vehicle to reposition to a global WGS84 location (VEHICLE_CMD_DO_REPOSITION) while respecting mode-executor ownership

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **ground_speed** | `double` | -1.000000 | Ground speed [m/s]. Negative selects the PX4 default. |
| **yaw** | `double` | ❌ | Desired yaw [°] clockwise from north. Leave empty to keep the current heading mode. |
| **altitude** | `double` | ❌ | Target altitude [m] (AMSL). |
| **longitude** | `double` | ❌ | Target longitude [°] (WGS84). |
| **latitude** | `double` | ❌ | Target latitude [°] (WGS84). |

### SendCmdSetNavState

**Plugin Class:** `auto_apms_px4::SendCmdSetNavState`

**C++ Model:** `auto_apms_px4::SendCmdSetNavState`

**Node Type:** `Action`

**Description:** Switches the active PX4 flight mode (by nav_state) while respecting mode-executor ownership

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **nav_state** | `int` | ❌ | Target PX4 navigation state (mode id) to switch to. |

### SendVehicleCommand

**Plugin Class:** `auto_apms_px4::SendVehicleCommand`

**C++ Model:** `auto_apms_px4::SendVehicleCommand`

**Node Type:** `Action`

**Description:** Publishes a raw PX4 VehicleCommand (command id + param1..param7) while respecting mode-executor ownership

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **target_component** | `int` | 0 | Component that should execute the command (0 = all). |
| **target_system** | `int` | 0 | System that should execute the command. |
| **confirmation** | `int` | 0 | Confirmation count (0 = first transmission of this command). |
| **param7** | `double` | 0.000000 | Command parameter 7. |
| **param6** | `double` | 0.000000 | Command parameter 6. |
| **param5** | `double` | 0.000000 | Command parameter 5. |
| **param4** | `double` | 0.000000 | Command parameter 4. |
| **param3** | `double` | 0.000000 | Command parameter 3. |
| **param2** | `double` | 0.000000 | Command parameter 2. |
| **param1** | `double` | 0.000000 | Command parameter 1. |
| **command** | `int` | ❌ | VehicleCommand command id (see px4_msgs/msg/VehicleCommand). |

### SetActuators

**Plugin Class:** `auto_apms_px4::SetActuatorsAction`

**C++ Model:** `auto_apms_px4::SetActuators`

**Node Type:** `Action`

**Description:** Sends direct actuator commands (motors and/or servos) to the vehicle

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **hold_period_ms** | `unsigned int` | 0 | Hold the commands for this duration [ms] before stopping. 0 = send once then stop immediately |
| **servo_commands** | `std::vector<float, std::allocator<float> >` | ❌ | Servo commands as a semicolon-separated list. Range: [-1, 1], 'nan'\|'NaN'\|'NAN' = disarmed |
| **motor_commands** | `std::vector<float, std::allocator<float> >` | ❌ | Motor commands as a semicolon-separated list. Range: [-1, 1], 'nan'\|'NaN'\|'NAN' = disarmed |
