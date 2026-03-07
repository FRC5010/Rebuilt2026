# JSON Schema Configuration Summary

This document describes all JSON schemas configured in VSCode for the FRC5010 project.

## Schemas Overview

All JSON schemas are stored in: `src/main/java/org/frc5010/common/config/schemas/`

### 1. **robot.json** → `robot.schema.json`
**Location:** `src/main/deploy/*/robot.json`

**Java Class:** `RobotJson.java`

**Purpose:** Main robot configuration file containing drivetrain settings, game piece definitions, and simulation options.

**Key Fields:**
- `userConfig` - Reference to user mode file (e.g., competition_mode.json)
- `driveType` - Drivetrain type (YAGSL_SWERVE_DRIVE, AKIT_SWERVE_DRIVE)
- `trackWidth`, `wheelBase`, `wheelDiameter` - Drivetrain dimensions with units
- `physicalMaxSpeed` - Maximum robot speed capability
- `driveMotorGearRatio` - Gear ratio between motor and wheels
- `loadSimulatedField` - Whether to load simulated field
- `gamePieceA`, `gamePieceB` - Game piece definitions

---

### 2. **competition_mode.json** / **demo_mode.json** → `user-mode.schema.json`
**Location:** `src/main/deploy/*/competition_mode.json`, `src/main/deploy/*/demo_mode.json`

**Java Class:** `UserModeJson.java`

**Purpose:** Defines operational limits for different robot modes (competition, demo, practice, etc.)

**Key Fields:**
- `maxSpeed` - Maximum teleop speed (m/s)
- `maxAngularSpeed` - Maximum angular velocity (rad/s)
- `maxAccelleration` - Maximum linear acceleration (m/s²)
- `maxAngularAccelleration` - Maximum angular acceleration (rad/s²)

---

### 3. **cameras.json** → `cameras.schema.json`
**Location:** `src/main/deploy/*/cameras.json`

**Java Class:** `CameraConfigurationJson.java`

**Purpose:** List of camera configuration files to load

**Key Fields:**
- `cameras` - Array of camera configuration filenames in the cameras/ directory

---

### 4. **cameras/*.json** → `camera.schema.json`
**Location:** `src/main/deploy/*/cameras/*.json` (e.g., intake.json, localization.json)

**Java Class:** `CameraConfigurationJson.java`

**Purpose:** Configuration for individual camera systems (PhotonVision, Limelight, etc.)

**Key Fields:**
- `name` - Unique camera identifier
- `use` - Use case (target, apriltag, quest)
- `type` - Camera type (limelight, photonvision)
- `strategy` - Pose estimation strategy
- `x`, `y`, `z` - Position offsets from robot center (meters)
- `roll`, `pitch`, `yaw` - Rotation angles (degrees)
- `width`, `height` - Camera resolution (pixels)
- `exposureTimeMs` - Exposure time in milliseconds; 0 = camera default / auto-exposure (PhotonVision only)
- `focalLengthX` - Horizontal focal length (fx) in pixels from calibration; 0 = use global `VisionConstants` default
- `focalLengthY` - Vertical focal length (fy) in pixels from calibration; 0 = fall back to `focalLengthX`
- `meanReprojectionError` - Mean pixel reprojection error from calibration (pixels); 0 = use global default

---

### 5. **controllers.json** → `controllers.schema.json`
**Location:** `src/main/deploy/*/controllers.json`

**Java Class:** `DriveteamControllersJson.java`

**Purpose:** List of driveteam controller configuration files

**Key Fields:**
- `controllers` - Array of controller configuration filenames in the controllers/ directory

---

### 6. **controllers/*.json** → `driveteam-controller.schema.json`
**Location:** `src/main/deploy/*/controllers/*.json` (e.g., driver.json, operator.json)

**Java Class:** `DriveteamControllerJson.java`

**Purpose:** Configuration for individual driveteam controllers (driver, operator, etc.)

**Key Fields:**
- `name` - Controller name (driver, operator, etc.)
- `port` - USB port number (0-5)
- `axis` - Array of axis configuration filenames in controllers/axis/ directory

---

### 7. **controllers/axis/*.json** → `driveteam-controller-axis.schema.json`
**Location:** `src/main/deploy/*/controllers/axis/*.json`

**Java Class:** `DriveteamControllerAxisJson.java`

**Purpose:** Configuration for individual joystick/controller axes

**Key Fields:**
- `channel` - Axis channel number (0-11)
- `deadband` - Minimum threshold to register motion (0.0-1.0)
- `invert` - Whether to invert axis
- `scale` - Output scaling factor
- `curvePower` - Curve power for non-linear response (1.0 = linear)
- `limit` - Maximum output limit (0.0-1.0)
- `rate` - Slew rate limiting

---

### 8. **robots.json** → `robots.schema.json`
**Location:** `src/main/deploy/robots.json`

**Java Class:** `RobotsJson.java`

**Purpose:** Top-level configuration mapping robot identities to robot classes

**Key Fields:**
- `competitionPin` - DIO pin for competition selector (-1 to disable)
- `robots` - Object mapping robot IDs to configurations
  - `id` - MAC address or unique identifier
  - `robotClass` - Fully qualified Java class name
  - `simulate` - Whether to use in simulation
  - `competition` - Whether to use when competition pin is active

---

### 9. **akit_swerve_drivetrain.json** → `akit_swerve_drivetrain.schema.json`
**Location:** `src/main/deploy/*/akit_swerve_drivetrain.json`

**Java Class:** `AKitSwerveDrivetrainJson.java`

**Purpose:** Detailed AKit swerve drivetrain configuration with motor control and module setup

**Key Fields:**
- `type` - Motor controller type (TalonFX, SparkTalon, Spark)
- `constants` - Physical and control constants
  - Dimensions and masses
  - Motor control parameters (PID, feedforward)
  - Module configurations (drive motor, steer motor, encoder setup)
  - CAN bus configuration

---

## Subsystem Configuration Schemas

### 10. **led_strip.json** → `led-strip.schema.json`
**Location:** `src/main/deploy/*/led_strip.json`

**Java Class:** `LEDStripConfigJson.java`

**Purpose:** Configuration for addressable LED strips (WS2812, APA102, etc.)

**Key Fields:**
- `length` - Total number of LEDs
- `dataPin` - PWM pin number for LED control
- `segments` - Array of named LED segments (e.g., status_indicator, team_colors)

---

### 11. **percent_motor.json** → `percent-motor.schema.json`
**Location:** `src/main/deploy/*/subsystems/example/percent_motor.json`

**Java Class:** `PercentMotorConfigurationJson.java`

**Purpose:** Configuration for a simple percent-output motor (no closed-loop control)

**Key Fields:**
- `name` - Subsystem name
- `controller` - Controller type (spark, talonfx, talonsrx, sparkmax)
- `type` - Motor type (Neo, KrakenX60, Falcon500)
- `id` - CAN ID
- `gearing` - Motor gearing ratio
- `momentOfInertiaKgMSq` - Moment of inertia for simulation
- `x`, `y`, `z` - Position for visualization (meters)
- `logLevel` - Telemetry logging level

---

### 12. **velocity_motor.json** → `velocity-motor.schema.json`
**Location:** `src/main/deploy/*/subsystems/example/velocity_motor.json`

**Java Class:** `VelocityMotorConfigurationJson.java`

**Purpose:** Configuration for a velocity-controlled motor with PID and feedforward

**Key Fields:**
- `name` - Subsystem name
- `controller` - Controller type
- `type` - Motor type
- `id` - CAN ID
- `gearing` - Motor gearing ratio
- `momentOfInertiaKgMSq` - Moment of inertia for simulation
- `x`, `y`, `z` - Position for visualization (meters)
- `kP`, `kI`, `kD` - PID gains
- `kS`, `kV`, `kA` - Feedforward constants
- `iZone` - PID integral zone
- `logLevel` - Telemetry logging level

---

### 13. **yams_arm.json** → `yams-arm.schema.json`
**Location:** `src/main/deploy/*/subsystems/example/yams_arm.json`

**Java Class:** `YamsArmConfigurationJson.java`

**Purpose:** Configuration for a YAMS robotic arm with angular positioning (joints, shoulders, etc.)

**Key Fields:**
- `motorSetup` - Motor controller configuration
- `motorSystemId` - PID and feedforward parameters
- `length` - Arm effective length
- `lowerHardLimit`, `upperHardLimit` - Hardware angle limits
- `lowerSoftLimit`, `upperSoftLimit` - Software angle limits
- `startingAngle` - Initial angle at startup
- `gearing` - Gear reduction stages
- `mass` - Arm mass
- `horizontalZero` - Angle at which arm is horizontal

---

### 14. **yams_elevator.json** → `yams-elevator.schema.json`
**Location:** `src/main/deploy/*/subsystems/example/yams_elevator.json`

**Java Class:** `YamsElevatorConfigurationJson.java`

**Purpose:** Configuration for a YAMS elevator mechanism with linear positioning

**Key Fields:**
- `motorSetup` - Motor controller configuration
- `motorSystemId` - PID and feedforward parameters
- `sprocketTeeth` - Number of sprocket teeth (for chain drive)
- `drumRadius` - Drum radius (for direct drive)
- `lowerHardLimit`, `upperHardLimit` - Hardware height limits (meters)
- `lowerSoftLimit`, `upperSoftLimit` - Software height limits (meters)
- `gearing` - Gear reduction stages
- `startingPosition` - Initial height at startup
- `mass` - Carriage and payload mass

---

### 15. **yams_pivot.json** → `yams-pivot.schema.json`
**Location:** `src/main/deploy/*/subsystems/example/yams_pivot.json`

**Java Class:** `YamsPivotConfigurationJson.java`

**Purpose:** Configuration for a YAMS pivot mechanism (turret rotation, wrist, etc.)

**Key Fields:**
- `motorSetup` - Motor controller configuration
- `motorSystemId` - PID and feedforward parameters
- `lowerHardLimit`, `upperHardLimit` - Hardware angle limits
- `lowerSoftLimit`, `upperSoftLimit` - Software angle limits
- `startingAngle` - Initial angle at startup
- `gearing` - Gear reduction stages
- `moi` - Moment of inertia

---

### 16. **yams_shooter.json** → `yams-shooter.schema.json`
**Location:** `src/main/deploy/*/subsystems/example/yams_shooter.json`

**Java Class:** `YamsShooterConfigurationJson.java`

**Purpose:** Configuration for a YAMS flywheel shooter mechanism

**Key Fields:**
- `motorSetup` - Motor controller configuration
- `motorSystemId` - PID and feedforward parameters
- `lowerSoftLimit`, `upperSoftLimit` - Velocity limits (RPM or deg/s)
- `gearing` - Gear reduction stages
- `mass` - Flywheel mass
- `diameter` - Flywheel diameter
- `moi` - Moment of inertia

---

## Shared Utility Schemas

These schemas are referenced by multiple configuration files:

### **unit-value.schema.json**
Represents a numeric value with unit of measure. Used for all physical measurements.

```json
{
  "val": 123.45,
  "uom": "meters"
}
```

### **motor-setup.schema.json`
Motor controller hardware configuration. Referenced by YAMS subsystem schemas.

**Fields:**
- `name` - Motor name
- `controllerType` - Controller type
- `motorType` - Motor model
- `canId` - CAN identifier
- `canBus` - CAN bus name (optional)
- `idleMode` - BRAKE or COAST
- `currentLimit` - Current limit with units
- `inverted` - Motor inversion flag
- `followers` - Array of follower motor configs

### **motor-system-id.schema.json**
PID and feedforward control parameters. Referenced by YAMS subsystem and velocity motor schemas.

**Fields:**
- `closedLoopRamp` - Ramp rate
- `openLoopRamp` - Ramp rate
- `feedBack` - PID constants (p, i, d)
- `feedForward` - FF constants (s, g, v, a)
- `maxVelocity` - Velocity limit
- `maxAcceleration` - Acceleration limit
- `controlMode` - CLOSED_LOOP or OPEN_LOOP

---

## VSCode Configuration

All schemas are configured in `.vscode/settings.json` under the `json.schemas` array. Each schema entry specifies:
- `fileMatch` - Glob pattern(s) to match against file paths
- `url` - Relative path to the schema file

## Usage

Once configured:
1. Open any JSON file matching one of the patterns above
2. VSCode will automatically provide:
   - **Intellisense/autocomplete** for available properties
   - **Validation** against the schema
   - **Hover documentation** for each field
   - **Error highlighting** for invalid values

## Adding New Schemas

To add a new schema:
1. Create the schema file in `src/main/java/org/frc5010/common/config/schemas/`
2. Add an entry to `.vscode/settings.json` under `json.schemas`
3. Reload VSCode window for changes to take effect

## Schema Testing

To test if schemas are working:
1. Open a JSON file matching one of the patterns
2. Check the VSCode status bar (bottom right)
3. You should see schema information or validation errors
4. Hover over properties to see documentation
