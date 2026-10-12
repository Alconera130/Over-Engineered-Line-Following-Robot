# ESP32-C3 Line Follower — Linea Viper X16

Firmware for the ESP32-C3 Mini, Linea Viper X16 reflectance array, and TB6612FNG differential-drive motor driver. The C3 profile matches the wiring recorded for this robot in `include/RobotConfig.h`.

## Features

- 16-channel mux scanning, per-sensor black/white calibration, filtering, centroid estimation, and persistent calibration in ESP32 NVS.
- Time-scaled PID with filtered derivative, bounded integral, anti-windup, and speed reduction as the line moves quickly or far from centre.
- Segment tracking to keep following the current line when the array sees multiple separate segments.
- A route array for wide junctions, with optional routing of confirmed one-sided 90-degree turns. Acute bends remain under continuous PID control when sharp-turn routing is disabled.
- Short broken-line bridging using the last steering correction, followed by a bounded search toward the last known line side.
- Optional track-time schedule for changing motor-command scale or stopping after elapsed time.
- Optional white-line mode, automatic black/white polarity switching, dead-end U-turns, and stop-line recognition.
- Release-triggered START behavior: hold for one second and release to calibrate; tap and release to start; tap and release while moving to stop. A stopped run begins again at the first route entry.

## File layout

Each behavior module has a header under its category in `include/` and an implementation under the matching category in `src/robot/`. `Robot.cpp` connects the modules into the run-state machine.

```text
include/
  Robot.h                         coordinator interface
  RobotConfig.h                   pins and tuning constants
  RobotTypes.h                    shared data types and states
  Diagnostics.h                  serial-port selection
  control/
    LineController.h              PID and curve speed profile
    MotorDriver.h                 PWM and slew limiting
  navigation/
    BrokenLineBridge.h            short-gap drive
    DeadEndReturn.h               optional reverse and timed U-turn
    JunctionHandler.h             junction detection and line reacquisition
    LineRecovery.h                lost-line search timing and motion
    RoutePlanner.h                route sequence and branch choice
    SegmentTracker.h              current-line segment lock
    SharpTurnHandler.h            one-sided sharp-turn detection
  sensors/
    ViperX16.h                    mux scan, sensor processing, calibration storage
  system/
    CalibrationSweep.h            calibration motion sequence
    StartButton.h                 debounce and release events
    TrackTimer.h                  elapsed track time and scheduled actions

src/robot/
  Robot.cpp                       state-machine coordinator
  control/                        matching control implementations
  navigation/                     matching navigation implementations
  sensors/                         ViperX16 implementation
  system/                          button, calibration, and track timer

src/viper_array_test.cpp           isolated sensor diagnostic; no motor control
```

The tunable values and pin map are in [`RobotConfig.h`](include/RobotConfig.h). [`ROBOT_CONSTANTS.txt`](ROBOT_CONSTANTS.txt) documents the active values and units.

## Wiring in this profile

| Signal | ESP32-C3 GPIO |
|---|---:|
| Viper SIGOUT | 4 (ADC) |
| Viper S0, S1, S2, S3 | 3, 2, 1, 5 |
| START button to GND | 9 |
| Built-in status LED | 8 (active low) |
| TB6612 AIN1, AIN2, PWMA | 7, 6, 10 |
| TB6612 BIN1, BIN2, PWMB | 20, 21, 0 |

The X16 appears to have its 16:1 mux on the board; the firmware treats SIGOUT and S0–S3 as that mux's common output and address pins. Confirm the board labels before connecting power. All ESP32 analog and logic signals must stay within 0–3.3 V. The built-in LED flashes briefly when the firmware recognizes a wide junction or a confirmed one-sided sharp turn; it is otherwise used for startup, calibration, recovery, and fault status.

GPIO2, GPIO8, and GPIO9 are ESP32-C3 boot-strapping pins. GPIO2 connects to mux S0, so use a pull-up that keeps it high during reset. GPIO8 also drives the common built-in active-low LED. GPIO9 is the START input; holding START while powering up or resetting can select the ROM download mode before firmware starts. GPIO20/21 are the C3 UART0 pins; this project's C3 profile routes `Serial` through native USB CDC so the motor direction signals are not shared with the console. GPIO0 carries PWMB but is not one of the C3's strapping pins.

### Power notes

- TB6612 `VCC` is logic power (3.3 V here); `VM` is the motor supply and must be wired separately to a supply suitable for the motors. The code cannot determine the unknown motor voltage or stall current. Confirm both from the motor label or datasheet before applying the 3-cell LiPo to `VM`; 2000 RPM alone does not identify a safe voltage.
- The buck converter's exact CN3903 board ratings vary. Verify that the specific module accepts a fully charged 3-cell LiPo input and can supply the ESP32, sensors, and driver logic without brownout. Join logic ground, sensor ground, and driver ground.
- Keep motor current paths away from the ADC/mux wiring. The bulk capacitor should be close to the motor driver's motor supply pins, with appropriate polarity and voltage rating. The 2200 µF capacitor across the battery terminals helps with transients; it does not replace local decoupling at the driver.
- A 1 kΩ resistor across an 11.1 V pack draws about 11 mA continuously (about 13 mA at 12.6 V), so check its power rating and whether that standby drain is intended.

## Build and operate

The default PlatformIO environment is `esp32-c3-devkitm-1`:

```powershell
platformio run -e esp32-c3-devkitm-1 --target upload
platformio device monitor -b 115200
```

### Sensor diagnostic

For mux and sensor troubleshooting, the separate `esp32-c3-viper-test` environment builds [`src/viper_array_test.cpp`](src/viper_array_test.cpp). It only drives the mux select pins and reads SIGOUT; keep motor power disconnected during this test.

```powershell
platformio run -e esp32-c3-viper-test --target upload
platformio device monitor -b 115200
```

The monitor streams raw readings from all 16 channels. Send `c` to clear the captured minimum/maximum values, move both line and floor under every sensor, then send `p` to print each channel's span. `OK` means the span meets `kMinimumCalibrationSpan`; `t` pauses or resumes the raw stream and `h` prints the commands.

If you already have raw readings and do not want to run the robot's calibration sweep, upload the robot firmware and connect USB while the robot is stopped. Send `m`, then enter one comma-separated row of 16 white-floor readings for channels 0 through 15, followed by one row of 16 black-line readings in the same order. Values are raw 12-bit ADC counts from 0 to 4095. The firmware validates the span and raw-reading direction, then saves the calibration in ESP32 NVS. It loads automatically on a later battery-powered boot, so the USB monitor is only needed for setup. Use the isolated sensor diagnostic above to collect readings; when taking black readings, place the black material under each channel in turn. With the current sensor direction setting, black readings should be higher than white readings.

On a fresh device, hold START for at least one second and release it. The robot sweeps in place for 4.8 seconds while all 16 channels record their low and high readings. Move the chassis so every sensor sees both the line and the floor. A channel whose recorded span is too small makes calibration fail. A successful calibration is stored in NVS; the robot will use it after restart.

After calibration, tap and release START to run. The track clock starts immediately when the run begins, so it includes the time spent driving out of a solid start marker. While it runs, tap and release START to stop. With path recovery disabled, a lost line is bridged for at most 110 ms, then the motors stop and the robot enters `FAULT`. Tap START once to resume with the saved route position, or double tap within 400 ms to restart the route sequence; hold START for one second and release to recalibrate. Console commands are `r` run, `c` calibrate, `m` manual calibration, `x` stop, `s` status, and `t` toggle telemetry. While connected over USB, enter `g P I D` and press Enter to change PID gains immediately (for example, `g 75 0 2.5`; each gain must be 0–500). This live command and telemetry require USB; battery-only runs use the configured gains. Gains reset to the configured defaults on the next run. Telemetry includes `tm` (elapsed track milliseconds), `v` (scheduled PID speed percentage), `kp`/`ki`/`kd` (active gains), and `vis` (whether the sensor currently detects a line); `l` and `r` remain unscaled PWM requests.

## Configuration

Edit `include/RobotConfig.h` and rebuild:

- `kDefaultLineIsDark = true` selects dark tape on a light floor. Set it to `false` for a light line on a dark floor.
- `kEnableAutomaticPolarity = false` leaves the selected polarity fixed. Set it to `true` only when the course changes line colour; the classifier then switches after several consistent frames.
- `kRoute` is the repeated action sequence for wide junctions and, optionally, one-sided sharp turns. Actions are `LEFT`, `RIGHT`, `STRAIGHT`, `ACUTE_LEFT`, and `ACUTE_RIGHT`. Acute actions take the matching branch with the stronger turn PWM values in `kAcuteTurnOutsidePwm` and `kAcuteTurnInsidePwm`; for example, change a route entry from `JunctionAction::RIGHT` to `JunctionAction::ACUTE_RIGHT` where the course needs a tighter right turn. They use `kAcuteMinimumTurnMs` before checking for the line and `kAcuteMaximumTurnMs` as their recovery timeout, separate from regular turn timing. If the requested branch is not visible, the planner falls back to a visible branch in deterministic left/straight/right order. The built-in LED gives one brief flash when a wide junction or confirmed sharp turn is detected; the flash does not indicate the selected direction.
- `kEnableJunctionRouting` is the default master switch for consuming `kRoute` entries. With `kStraightThroughWhenJunctionRoutingDisabled = true`, disabling routing crosses detected wide junctions and confirmed edge turns straight for the configured entry/traverse durations, without consuming the route list. Set that option to `false` to leave the robot under PID instead. Track-time events can toggle routing during a run.
- `kEnableSharpTurnRouting = false` leaves confirmed one-sided edge turns under PID control, so acute bends do not trigger a timed route maneuver. The detector still flashes the LED. Set it to `true` to let those turns consume `kRoute` entries; wide junction routing is controlled separately by `kEnableJunctionRouting`. The detector confirms three consecutive edge frames and begins at position 5200. Raise `kSharpTurnConfirmFrames` to reduce false triggers on zigzags; lower it if true 90-degree turns are missed.
- `kEnableTrackTimeActions` enables the ordered `kTrackTimeEvents` schedule. Time starts when a run begins, including start-zone exit. `SET_SPEED_PERCENT` scales PID following and short-gap bridge commands; it stays active until another speed event. `SET_PID_GAINS` sets P, I, and D together at the scheduled time, clears the old integral accumulator, and keeps the new gains active until the run ends; the next run resets to `kPidP`, `kPidI`, and `kPidD`. `ENABLE_JUNCTION_ROUTING` and `DISABLE_JUNCTION_ROUTING` toggle route selection at the scheduled time. With `kStraightThroughWhenJunctionRoutingDisabled = true`, disabling routing sends a detected branch straight through without consuming a route entry; an in-progress route turn switches to straight. `STOP` stops the run and returns to `WAIT_START`. Times must be increasing, and the timer resets on every new run. The schedule runs only when `kEnableTrackTimeActions` is true.
- `kEnablePathRecovery = false` stops the motors after the broken-line bridge expires; set it to `true` to search toward the last seen line instead.
- `kEnableDeadEndReturn = false` by default. If enabled while path recovery is on, a longer loss triggers a brief reverse, then a timed 180-degree search and return. At the previous junction the planner avoids repeating the last stored branch when another visible branch exists. Tune the duration carefully; without encoders this is approximate.
- `kEnableAutomaticPolarity`, `kEnablePathRecovery`, and `kEnableStopLine` are independent options. Dead-end return requires path recovery to be enabled.
- Tune motor minimum PWM to the actual wheel deadband. With the current `kMotorMinimumPwm = 20`, any smaller nonzero PWM command is raised to 20. The same floor limits timed slowdowns if scaled commands fall below 20.
- The default PID starting point is P=75, I=0, D=2.5. Tune on the final chassis and battery: keep I at zero, adjust P until steering is firm without weaving, then add D to damp the weave. Only add a small I for persistent bias after checking alignment. While running, use the `g P I D` console command to try gains without rebuilding. See `ROBOT_CONSTANTS.txt` for units and the configuration list.

## What the controller can and cannot infer

The X16 provides a transverse line position, not a map or a direct heading measurement. It can follow curved and zigzagging paths by steering on the changing line centroid; those changes do not consume route entries unless a junction or sustained one-sided edge meets its detector. Broken-line bridging predicts the near-future line position by briefly reusing the last steering correction.

Recovery rotates toward the last observed side and looks for a line. Optional dead-end return is open-loop and depends on time, motor voltage, wheel traction, and chassis geometry. There are no wheel encoders or inertial sensor in the listed hardware, so the robot cannot measure distance, verify a precise 180-degree rotation, or build a globally reliable map of repeated intersections. Wheel encoders and a per-wheel speed loop are the most useful next additions for repeatable turns and speed control.

## Differential-drive kinematics

The Viper centroid is negative when the line is left of the sensor centre and positive when it is right. The controller adds correction to one wheel and subtracts it from the other. For wheel linear speeds `vL` and `vR`, wheel separation `b`, and chassis forward speed `v`, the ideal no-slip differential-drive model is:

```text
forward speed v = (vR + vL) / 2
yaw rate        ω = (vR - vL) / b
turn radius     R = v / ω = b (vR + vL) / (2 (vR - vL))
```

These equations use wheel speed, not PWM. PWM is only an open-loop voltage request, so battery sag, motor mismatch, floor grip, and load alter the true radius. The slew limiter softens command changes, while the PID curvature profile reduces forward PWM during rapid or large errors. Mount the sensor rigidly and slightly ahead of the axle: more forward offset gives earlier warning of a curve but makes the robot more sensitive to lateral line motion.
# Over-Engineered-Line-Following-Robot
