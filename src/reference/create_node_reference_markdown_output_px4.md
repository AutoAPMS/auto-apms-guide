<!-- markdownlint-disable MD024 MD041 MD060 -->
| Registration Name | Class Name | Package |
| :--- | :---: | :---: |
| [Arm](#arm) | `auto_apms_px4_behavior::ArmAction` | auto_apms_px4_behavior |
| [Disarm](#disarm) | `auto_apms_px4_behavior::DisarmAction` | auto_apms_px4_behavior |
| [EnableHold](#enablehold) | `auto_apms_px4_behavior::EnableHoldAction` | auto_apms_px4_behavior |
| [GetGlobalPosition](#getglobalposition) | `auto_apms_px4_behavior::GetPosition<GlobalPositionMsg>` | auto_apms_px4_behavior |
| [GetLocalPosition](#getlocalposition) | `auto_apms_px4_behavior::GetPosition<LocalPositionMsg>` | auto_apms_px4_behavior |
| [GoTo](#goto) | `auto_apms_px4_behavior::GoToAction` | auto_apms_px4_behavior |
| [GoToGlobal](#gotoglobal) | `auto_apms_px4_behavior::GoToAction` | auto_apms_px4_behavior |
| [GoToLocal](#gotolocal) | `auto_apms_px4_behavior::GoToAction` | auto_apms_px4_behavior |
| [Land](#land) | `auto_apms_px4_behavior::LandAction` | auto_apms_px4_behavior |
| [Mission](#mission) | `auto_apms_px4_behavior::MissionAction` | auto_apms_px4_behavior |
| [RTL](#rtl) | `auto_apms_px4_behavior::RTLAction` | auto_apms_px4_behavior |
| [SetActuators](#setactuators) | `auto_apms_px4_behavior::SetActuatorsAction` | auto_apms_px4_behavior |
| [Takeoff](#takeoff) | `auto_apms_px4_behavior::TakeoffAction` | auto_apms_px4_behavior |

## auto_apms_px4_behavior

### Arm

**Plugin Class:** `auto_apms_px4_behavior::ArmAction`

**C++ Model:** `auto_apms_px4_behavior::Arm`

**Node Type:** `Action`

**Description:** Arms the vehicle, making it ready for takeoff

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **wait_until_ready_to_arm** | `bool` | false | Wait for the UAV to be ready for arming. If false and UAV is not ready to arm, will be rejected. |

### Disarm

**Plugin Class:** `auto_apms_px4_behavior::DisarmAction`

**C++ Model:** `auto_apms_px4_behavior::Disarm`

**Node Type:** `Action`

**Description:** Disarms the vehicle if it is landed

*This node doesn't have any ports.*

### EnableHold

**Plugin Class:** `auto_apms_px4_behavior::EnableHoldAction`

**C++ Model:** `auto_apms_px4_behavior::EnableHold`

**Node Type:** `Action`

**Description:** Enables or disables the position loiter mode

*This node doesn't have any ports.*

### GetGlobalPosition

**Plugin Class:** `auto_apms_px4_behavior::GetPosition<GlobalPositionMsg>`

**C++ Model:** `auto_apms_px4_behavior::GetGlobalPosition`

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

**Plugin Class:** `auto_apms_px4_behavior::GetPosition<LocalPositionMsg>`

**C++ Model:** `auto_apms_px4_behavior::GetLocalPosition`

**Node Type:** `Condition`

**Description:** Gets the current position of the vehicle in the local coordinate frame

#### Output Ports

| Output Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **down** | `double` | {down} | Current down [m] relative to origin |
| **east** | `double` | {east} | Current east [m] relative to origin |
| **north** | `double` | {north} | Current north [m] relative to origin |
| **vector** | `Eigen::Matrix<double, -1, -1, 0, -1, -1>` | {pos_vec} | Current local position vector (north [m], east [m], down [m]) |

### GoTo

**Plugin Class:** `auto_apms_px4_behavior::GoToAction`

**C++ Model:** `auto_apms_px4_behavior::GoTo`

**Node Type:** `Action`

**Description:** Commands the vehicle to go to a specified position. The reference frame can be set via the 'frame' input port

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **reached_thresh_yaw** | `double` | 7.000000 | Maximum heading error [°] under which the position is considered reached |
| **reached_thresh_vel** | `double` | 0.300000 | Maximum velocity error [m/s] under which the position is considered reached |
| **reached_thresh_pos** | `double` | 0.500000 | Maximum position error [m] under which the position is considered reached |
| **max_heading_rate** | `double` | 30.000000 | Maximum heading rate [°/s] |
| **max_vertical_vel** | `double` | 5.000000 | Maximum vertical velocity [m/s] |
| **max_horizontal_vel** | `double` | 10.000000 | Maximum horizontal velocity [m/s] |
| **yaw** | `double` | ❌ | Desired yaw position in degree from north (heading) [-180°, 180) |
| **z** | `double` | ❌ | Override vector entry Z |
| **y** | `double` | ❌ | Override vector entry Y |
| **x** | `double` | ❌ | Override vector entry X |
| **vector** | `Eigen::Matrix<double, -1, -1, 0, -1, -1>` | ❌ | Target position as a row vector (separated by ';') |
| **frame** | `std::string` | global | Reference frame: 'global' (Latitude, longitude, altitude (AMSL)) or 'local' (North, east, down from start) |

### GoToGlobal

**Plugin Class:** `auto_apms_px4_behavior::GoToAction`

**C++ Model:** `auto_apms_px4_behavior::GoToGlobal`

**Node Type:** `Action`

**Description:** Commands the vehicle to go to a specified position in the global coordinate frame

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **reached_thresh_yaw** | `double` | 7.000000 | Maximum heading error [°] under which the position is considered reached |
| **reached_thresh_vel** | `double` | 0.300000 | Maximum velocity error [m/s] under which the position is considered reached |
| **reached_thresh_pos** | `double` | 0.500000 | Maximum position error [m] under which the position is considered reached |
| **max_heading_rate** | `double` | 30.000000 | Maximum heading rate [°/s] |
| **max_vertical_vel** | `double` | 5.000000 | Maximum vertical velocity [m/s] |
| **max_horizontal_vel** | `double` | 10.000000 | Maximum horizontal velocity [m/s] |
| **yaw** | `double` | ❌ | Desired yaw position in degree from north (heading) [-180°, 180) |
| **z** | `double` | ❌ | Override vector entry Z |
| **y** | `double` | ❌ | Override vector entry Y |
| **x** | `double` | ❌ | Override vector entry X |
| **vector** | `Eigen::Matrix<double, -1, -1, 0, -1, -1>` | ❌ | Target position as a row vector (separated by ';') |
| **frame** | `std::string` | global | Reference frame: 'global' (Latitude, longitude, altitude (AMSL)) or 'local' (North, east, down from start) |

### GoToLocal

**Plugin Class:** `auto_apms_px4_behavior::GoToAction`

**C++ Model:** `auto_apms_px4_behavior::GoToLocal`

**Node Type:** `Action`

**Description:** Commands the vehicle to go to a specified position in the local coordinate frame

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **reached_thresh_yaw** | `double` | 7.000000 | Maximum heading error [°] under which the position is considered reached |
| **reached_thresh_vel** | `double` | 0.300000 | Maximum velocity error [m/s] under which the position is considered reached |
| **reached_thresh_pos** | `double` | 0.500000 | Maximum position error [m] under which the position is considered reached |
| **max_heading_rate** | `double` | 30.000000 | Maximum heading rate [°/s] |
| **max_vertical_vel** | `double` | 5.000000 | Maximum vertical velocity [m/s] |
| **max_horizontal_vel** | `double` | 10.000000 | Maximum horizontal velocity [m/s] |
| **yaw** | `double` | ❌ | Desired yaw position in degree from north (heading) [-180°, 180) |
| **z** | `double` | ❌ | Override vector entry Z |
| **y** | `double` | ❌ | Override vector entry Y |
| **x** | `double` | ❌ | Override vector entry X |
| **vector** | `Eigen::Matrix<double, -1, -1, 0, -1, -1>` | ❌ | Target position as a row vector (separated by ';') |
| **frame** | `std::string` | local | Reference frame: 'global' (Latitude, longitude, altitude (AMSL)) or 'local' (North, east, down from start) |

### Land

**Plugin Class:** `auto_apms_px4_behavior::LandAction`

**C++ Model:** `auto_apms_px4_behavior::Land`

**Node Type:** `Action`

**Description:** Commands the vehicle to land at its current position

*This node doesn't have any ports.*

### Mission

**Plugin Class:** `auto_apms_px4_behavior::MissionAction`

**C++ Model:** `auto_apms_px4_behavior::Mission`

**Node Type:** `Action`

**Description:** Starts a mission that has been previously uploaded to the vehicle

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **do_restart** | `bool` | false | Wether to restart (true) or resume (false) the mission. |

### RTL

**Plugin Class:** `auto_apms_px4_behavior::RTLAction`

**C++ Model:** `auto_apms_px4_behavior::RTL`

**Node Type:** `Action`

**Description:** Commands the vehicle to return to its home position and land

*This node doesn't have any ports.*

### SetActuators

**Plugin Class:** `auto_apms_px4_behavior::SetActuatorsAction`

**C++ Model:** `auto_apms_px4_behavior::SetActuators`

**Node Type:** `Action`

**Description:** Sends direct actuator commands (motors and/or servos) to the vehicle

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **hold_period_ms** | `unsigned int` | 0 | Hold the commands for this duration [ms] before stopping. 0 = send once then stop immediately |
| **servo_commands** | `std::vector<float, std::allocator<float> >` | ❌ | Servo commands as a semicolon-separated list. Range: [-1, 1], 'nan'|'NaN'|'NAN' = disarmed |
| **motor_commands** | `std::vector<float, std::allocator<float> >` | ❌ | Motor commands as a semicolon-separated list. Range: [-1, 1], 'nan'|'NaN'|'NAN' = disarmed |

### Takeoff

**Plugin Class:** `auto_apms_px4_behavior::TakeoffAction`

**C++ Model:** `auto_apms_px4_behavior::Takeoff`

**Node Type:** `Action`

**Description:** Commands the vehicle to take off to a specified altitude

#### Input Ports

| Input Name | Type | Default Value | Description |
| :--- | :---: | :---: | :--- |
| **alt** | `double` | ❌ | Target takeoff altitude in meter (AMSL) |
