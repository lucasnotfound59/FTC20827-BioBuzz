# FTC-20827 BioBuzz TeamCode

Team-owned FTC code for BIOBUZZ. The command architecture uses SolversLib's official command system (`org.solverslib:core:0.3.6`, plus `org.solverslib:pedroPathing:0.3.6` for Pedro Pathing 3 commands in autonomous), following its subsystem, command, and `CommandOpMode` lifecycle model.

Compiling proves the code builds. It does not prove deployment, sensor calibration or robot behavior: every item marked "verify on the robot" below still has to be checked on the real robot.

## Source layout

- `teleops/`: `BaseDriveTeleOp` holds the shared hardware, controls and lifecycle; `RobotCentricDrive` and `FieldCentricDrive` are the Driver Station entries.
- `autos/`: `BaseAuto`, the shared lifecycle and helpers for Pedro autonomous routines. There is no concrete autonomous OpMode yet.
- `subsystems/`: hardware ownership. `Drive` (four drive motors), `Shooter`, `Intake`, `Gate`, `Localizer` (OctoQuad heading), `AutoAim` (Limelight), `PathFollower` (Pedro follower), plus `GamepadConfig` for the control mapping.
- `commands/`: `DriveCommand`, the drivetrain default command for both TeleOps.
- `pedro/`: `Constants`, the Pedro Pathing 3 follower configuration.
- `utils/`: `Alliance` (field frame, alliance conversion, AprilTag IDs) and `AutoHandoff` (AUTO result for the next TeleOp).
- `tests/`: Driver Station diagnostic OpModes.

## Hardware names

Motors `frontLeft`, `frontRight`, `backLeft`, `backRight`, `shooterLeft`, `shooterRight`, `intake`; servo `gate`; OctoQuad MK2 `octoquad` (type OctoQuadFTC); Limelight 3A `limelight`.

The drive motor names and directions live in one place, the constants at the top of `Drive`. `pedro/Constants` reads the same constants, so the TeleOps and the Pedro follower always agree. Positive power on every wheel must roll the robot forward.

## Controls (gamepad 1)

| Input | Action |
| --- | --- |
| Left stick | Translate |
| Right stick X | Turn (scaled by `GamepadConfig.TURN_LIMIT`) |
| LB (hold) | Intake. The gate opens only while LB is held and the shooter is at speed |
| LT (hold) | Outtake |
| RB (hold) | Shooter |
| RT (hold) | Auto-aim steers the chassis (Field-Centric only) |
| BACK / SHARE | Make the current facing field-centric forward (Field-Centric only) |
| X / B during INIT | Select BLUE / RED for auto-aim (Field-Centric only) |

## Field-Centric TeleOp

The left stick translates relative to a fixed forward direction, however the robot is turned. After a 90 degree counterclockwise turn, stick-left still moves the robot to the driver's left.

The forward direction comes from one of three places:

1. Straight after an AUTO (within 30 s), from the AUTO's final pose and alliance: stick-forward is away from the drivers. This assumes the robot was not moved between AUTO and the TeleOp's INIT.
2. Otherwise, the robot's facing at START. Point the front away from the drivers before START.
3. BACK re-zeroes it to the current facing at any time.

Keep the robot still during INIT while the OctoQuad calibrates, and wait for `RUNNING`.

A single bad OctoQuad packet reuses the last good heading for up to 0.1 s (`Localizer.HEADING_HOLD_SECONDS`). If the heading stays unavailable, the sticks become robot-centric instead of stopping the robot, the gamepad rumbles, and telemetry shows `ROBOT-CENTRIC: heading unavailable`. Field-centric resumes when the heading returns.

### Auto-aim

`AutoAim` owns the Limelight. Each new camera frame that shows AprilTags under one of our HIVE CELLS (RED 30-37, BLUE 38-45) sets a target heading; between frames the OctoQuad heading closes the loop. While RT is held and a target is fresh, auto-aim replaces the right stick's turn and the left stick keeps translating field-centric. Moving the right stick past `AutoAim.MANUAL_OVERRIDE`, releasing RT, or losing the tags for `LOCK_HOLD_SECONDS` returns the turn to the driver.

Auto-aim does nothing until an alliance is known: it is taken from the AUTO just run, or selected with X / B during INIT. A robot configuration without `limelight` only disables auto-aim.

Verify on the robot before relying on it, at low `MAX_TURN`:

1. A tag to the right of the crosshair reads positive tx.
2. The robot turns toward the tag. If it turns away, set `AutoAim.INVERT_TURN` in Panels.
3. Calibrate `AIM_TX_DEG` (the tx at which shots land) and tune `TURN_KP`, `TURN_KS`, `TOLERANCE_DEG`.

`ALIGNED` is an angle check only. It does not know the range, the flywheel speed or which CELL is tilted up, and it does not gate the shooter.

## Autonomous base

`BaseAuto` is written against the BIOBUZZ Competition Manual TU08. Subclasses pass an `Alliance`, implement `startPose()` and `buildRoutine()`, and write every pose for the BLUE side with `pose(x, y, headingDegrees)`.

Field frame (a team convention, documented in `utils/Alliance`): inches, origin at the audience-side corner next to the RED alliance area, +x toward the BLUE alliance wall, +y away from the audience, heading 0 along +x and counterclockwise-positive. BLUE owns x > 72. RED is the BLUE routine rotated 180 degrees about the field centre, because the field is rotationally symmetric (LOADING ZONES on tiles A5 and F2). Check the frame against a real field before trusting any pose.

What the base does for the rules:

- G304: nothing drives before START; the start pose is re-applied at START in case the robot was nudged. Place the robot on its own side, touching the wall, outside the LOADING ZONE, with 4 POLLEN.
- G402: a pose within `OWN_SIDE_MARGIN_IN` of the centre line shows a warning on the Driver Station.
- G403: everything stops `END_GUARD_SECONDS` before the 30 s mark and again when the OpMode ends.
- LEAVE 3, PARK 5, HIVE TIP 20: `untilSecondsLeft(command, seconds)` interrupts scoring so the final LEAVE / PARK move always gets its time.
- `shoot(feedSeconds)` spins up, opens the gate and feeds, and stops intake, gate and flywheel even when interrupted.

Before any path is driven, `pedro/Constants` still needs real values: the OctoQuad pod ports, directions and offsets are placeholders, and the Foresight block is Pedro's documentation sample. Pedro's AutoTune produces them, but it needs the Quickstart's `pedro/Tuning.java` and `pedro/procedures` copied into this project first.

## Dependencies

SolversLib `0.3.6`, Pedro Pathing `revhub:3.0.0` and `tuning:1.0.0`, FTC SDK `12.0.0`. `TeamCode/build.gradle` excludes only SolversLib's transitive `org.firstinspires.ftc` SDK artifacts because its strict 11.1 constraints conflict with the application's explicit SDK 12.0 dependencies.

## Reference

- [SolversLib command system](https://docs.seattlesolvers.com/command-base/command-system)
- [SolversLib CommandOpMode](https://docs.seattlesolvers.com/command-base/command-system/robot-and-commandopmode)
- [SolversLib installation](https://docs.seattlesolvers.com/installation)
- [Pedro Pathing 3 constants](https://pedropathing.com/docs/pathing/tuning/constants) and [OctoQuad localizer](https://pedropathing.com/docs/pathing/tuning/localization/octoquad)
- [BIOBUZZ game and season materials](https://ftc-resources.firstinspires.org/ftc/game)
- [Limelight FTC programming guide](https://docs.limelightvision.io/docs/docs-limelight/apis/ftc-programming)
