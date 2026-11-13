# FTC-20827 BioBuzz TeamCode

Team-owned FTC code for BIOBUZZ. The command architecture uses SolversLib's official command system (`org.solverslib:core:0.3.6`, plus `org.solverslib:pedroPathing:0.3.6` for Pedro Pathing 3 commands in autonomous), following its subsystem, command, and `CommandOpMode` lifecycle model.

Compiling proves the code builds. It does not prove deployment, sensor calibration or robot behavior: every item marked "verify on the robot" below still has to be checked on the real robot.

## Source layout

- `teleops/`: `BaseDriveTeleOp` holds the shared hardware, controls and lifecycle; `PedroDriveTeleOp` adds the shared Pedro follower, B/X paths, AUTO handoff and odometry reset; `RobotCentricDrive` and `FieldCentricDrive` are the Driver Station entries.
- `autos/`: `BaseAuto`, the shared lifecycle and helpers for Pedro autonomous routines, and `RedMainAuto`, the RED autonomous.
- `subsystems/`: hardware ownership. `Drive` (four drive motors), `Shooter`, `Intake`, `Gate`, `Localizer` (OctoQuad heading), `PathFollower` (Pedro follower), plus `GamepadConfig` for the control mapping.
- `commands/`: `DriveCommand`, a plain drivetrain default command (no TeleOp uses it now; both drive through Pedro).
- `pedro/`: `Constants`, the Pedro Pathing 3 follower configuration; `Tuning`, which registers Pedro's AutoTune procedures; `procedures/`, unmodified copies of the four Quickstart procedures this robot needs.
- `utils/`: `Alliance` (field frame, alliance conversion, AprilTag IDs) and `AutoHandoff` (AUTO result for the next TeleOp).
- `tests/`: Driver Station diagnostic OpModes.

## Hardware names

Motors `frontLeft`, `frontRight`, `backLeft`, `backRight`, `shooterLeft`, `shooterRight`, `intake`; positional servo `gate`; continuous-rotation servo `splitter`; OctoQuad MK2 `octoquad` (type OctoQuadFTC).

Configure `splitter` as a Servo in Robot Configuration, with the physical servo in continuous-rotation mode. `Servo.setPosition()` supplies rotation commands, not angles: deploy defaults to 0.75, retract to 0.25, and neutral to 0.5 (direction is FORWARD). Calibrate `Intake.SPLITTER_DEPLOY_POSITION`, `SPLITTER_RETRACT_POSITION`, `SPLITTER_STOP_POSITION`, and the separate `SPLITTER_DEPLOY_SECONDS` / `SPLITTER_RETRACT_SECONDS` travel times in Panels. Both times default to zero, disabling movement until calibrated. Initialization sends neutral. Each accepted A press switches deploy/retract direction and starts its timer; expiry sends neutral without blocking driving. Presses during movement are discarded, requiring a release and a fresh press afterward. Invalid or disabled movements do not toggle the target. OpMode STOP also sends neutral. FLOWER collection uses the deploy timer and sends neutral when finished or interrupted.

The drive motor names and directions live in one place, the constants at the top of `Drive`. `pedro/Constants` reads the same constants, so the TeleOps and the Pedro follower always agree. Positive power on every wheel must roll the robot forward. In the TeleOps, wheel commands use normalized motor power without drive motor encoders. The Pedro follower also sends power directly.

## Controls

Gamepad 1 (driver):

| Input | Action |
| --- | --- |
| Left stick | Translate (scaled by `GamepadConfig.DRIVE_LIMIT`) |
| Right stick X | Turn (scaled by `GamepadConfig.TURN_LIMIT`) |
| LB (hold) | Intake |
| LT (hold) | Outtake |
| RT (hold) | Fire, same as gamepad 2 |
| A (press) | Splitter down / up |
| B (press) | Pedro drives to shoot1 |
| X (press) | Pedro drives to shoot2 |
| D-pad down (press) | Reset odometry to the alliance's reset pose |

Gamepad 2 (operator):

| Input | Action |
| --- | --- |
| LB (hold) | Intake, same as gamepad 1 |
| LT (hold) | Outtake, same as gamepad 1 |
| RB (hold) | Shooter spin-up |
| RT (hold) | Fire: runs the intake at full power, then opens the gate once the intake has spun up and the shooter is at speed |

Intake, outtake and fire work from either gamepad; if one holds intake and the other outtake, intake wins. To shoot: gamepad 2 holds RB to spin up and either gamepad holds RT to fire. Fire runs the intake at full power at once (overriding intake/outtake), and opens the gate only after `Intake.FIRE_SPINUP_SECONDS` (the intake has no encoder, so this time stands in for reaching full speed) and while the shooter is at speed.

## Field-Centric and Robot-Centric TeleOps

Both run on `PedroDriveTeleOp`: one Pedro follower for manual drive, the B/X paths and the OctoQuad odometry, so the pose, the B/X shooting points, the AUTO handoff and the reset work the same in both, and their settings live in Panels under `PedroDriveTeleOp`. Field-Centric drives relative to the field (below); Robot-Centric drives relative to the robot's facing.

Manual driving and paths use the same field pose as AUTO: inches, with heading in radians and counterclockwise-positive. Stick-forward follows field +x; stick-left follows field +y. No alliance or driver-forward offset is applied to manual controls.

Handoff: when an AUTO ended less than 30 s ago, INIT starts from the odometry pose where that AUTO ended, with its alliance (`USE_AUTO_HANDOFF`). The Driver Station shows that pose during INIT. Re-INITing the TeleOp picks it up again. Otherwise, set `START_X_IN`, `START_Y_IN`, `START_HEADING_DEG`, and `START_ON_BLUE` before INIT. Keep the robot still during INIT while Pedro initializes the OctoQuad.

Reset: gamepad 1 D-pad down overwrites the odometry pose with `RED_RESET_X_IN` / `RED_RESET_Y_IN` / `RED_RESET_HEADING_DEG` or the `BLUE_RESET_*` values, by the current alliance, cancels a running B/X path, and rumbles gamepad 1. The reset values are placeholders (0) until measured.

Press B for shoot1 or X for shoot2. Moving a drive stick cancels the path. One Pedro follower handles manual drive, path following, and localization; the standalone `Localizer` subsystem is not constructed in this TeleOp.

## Autonomous base

`BaseAuto` is written against the BIOBUZZ Competition Manual TU08. Subclasses pass an `Alliance`, implement `startPose()` and `buildRoutine()`, and write every pose for the BLUE side with `pose(x, y, headingDegrees)`.

Field frame (a team convention, documented in `utils/Alliance`): inches, origin at the audience-side corner next to the RED alliance area, +x toward the BLUE alliance wall, +y away from the audience, heading 0 along +x and counterclockwise-positive. BLUE owns x > 70.75 (Pedro's field is 141.5 in square, `Alliance.FIELD_SIZE`). RED is the BLUE routine rotated 180 degrees about the field centre, because the field is rotationally symmetric (LOADING ZONES on tiles A5 and F2). Check the frame against a real field before trusting any pose.

What the base does for the rules:

- G304: nothing drives before START; the start pose is re-applied at START in case the robot was nudged. Place the robot on its own side, touching the wall, outside the LOADING ZONE, with 4 POLLEN.
- G402: a pose within `OWN_SIDE_MARGIN_IN` of the centre line shows a warning on the Driver Station.
- G403: everything stops `END_GUARD_SECONDS` before the 30 s mark and again when the OpMode ends.
- LEAVE 3, PARK 5, HIVE TIP 20: `untilSecondsLeft(command, seconds)` interrupts scoring so the final LEAVE / PARK move always gets its time.
- `shoot(feedSeconds)` works like the TeleOp fire button: it spins up the flywheel and runs the intake at full power with the gate closed, opens the gate once the intake has had `Intake.FIRE_SPINUP_SECONDS` and the flywheel is at speed, feeds, and stops intake, gate and flywheel even when interrupted.

Before any path is driven, `pedro/Constants` still needs real values: the OctoQuad pod ports, directions and offsets are placeholders, and the Foresight block is Pedro's documentation sample. See the next section.

## Red Main Auto

`autos/RedMainAuto` (Driver Station: **Red Main Auto**) drives the eleven paths of `Auto routines/Auto red v1。1.pp.json` in order: an opening line from the wall to the shooting position, then the ten called Path 1-10 below. Its coordinates are the RED-side numbers from that file; a private `red(...)` helper converts them to the BLUE poses `BaseAuto` expects.

It shoots the preload at the shooting position before Path 1 and shoots again after Paths 3, 7 and 9 (`SHOOT_SECONDS` of feeding each), runs the intake along Path 2, and collects at a FLOWER at the end of Paths 5 and 8 (`FLOWER_SECONDS` stopped with the intake on). The splitter deploys at each FLOWER and retracts after collection; interruption stops it. Everything before the last path is cut off `PARK_RESERVE_SECONDS` before the end so the drive into the RED LOADING ZONE always gets its time.

It has not been run on the robot, and at the default path speed the plan is longer than 30 s, so its end is cut off.

## Pedro tuning (AutoTune)

Deploy, connect to the Robot Controller's Wi-Fi and open `http://192.168.43.1:10158`. Run the procedures in this order, pasting each generated block over the matching one in `pedro/Constants` and redeploying before the next step:

1. **Mecanum Tuner**, wheels off the ground. It confirms motor names and directions. The result must equal the constants in `Drive`; if a wheel is wrong, fix it in `Drive`.
2. **OctoQuad Tuner**, pushed by hand with no motor power. Produces `localizerConfig`.
3. **Tests, Odometry Test**. The robot drives itself forward, left and counterclockwise at half power. Do not continue until it passes.
4. **Foresight Tuner**. The robot drives and spins at full power and coasts past its target: clear floor, one person on the Driver Station STOP. Produces `foresightConfig`.
5. **Tests, Hold / Line / Curve**, to check the finished follower.

The generated blocks declare `public static` fields with the same names as the existing ones, so they replace them directly. Keep `c.manualBrakeMode.set(true)` in `drivetrainConfig` and keep the `Drive.*` constants rather than the literal names AutoTune prints.

## Sloth hot reload

[Sloth](https://github.com/Dairy-Foundation/Sloth) `0.3.2` sends only the `org.firstinspires.ftc.teamcode` classes to the robot, in about a second, and applies them when the running OpMode ends.

- First do one normal install (the **TeamCode** run configuration) so the robot has the Sloth runtime.
- After that, run the **deploySloth** run configuration (`.run/deploySloth.run.xml`), or `./gradlew :TeamCode:deploySloth`, for code changes inside TeamCode.
- Do a normal install again after changing a dependency or anything outside TeamCode. Run `./gradlew :TeamCode:removeSlothRemote` first, or add it as a "before launch" Gradle task of the TeamCode run configuration; otherwise the hot-loaded code still on the robot replaces the newly installed code.

Panels `1.0.20` and Pedro tuning `1.0.3` depend on this same Sloth version. Not tried on the robot yet.

## Dependencies

SolversLib `0.3.6`, Pedro Pathing `revhub:3.0.2` and `tuning:1.0.3` (the versions the official Quickstart pins; 3.0.2 fixes the OctoQuad pod offsets being applied to the wrong axes in 3.0.0), FTC SDK `12.0.0`. SolversLib's `pedroPathing:0.3.6` was built against `revhub:3.0.0`; every Pedro method it calls still exists in 3.0.2. `TeamCode/build.gradle` excludes only SolversLib's transitive `org.firstinspires.ftc` SDK artifacts because its strict 11.1 constraints conflict with the application's explicit SDK 12.0 dependencies.

## Reference

- [SolversLib command system](https://docs.seattlesolvers.com/command-base/command-system)
- [SolversLib CommandOpMode](https://docs.seattlesolvers.com/command-base/command-system/robot-and-commandopmode)
- [SolversLib installation](https://docs.seattlesolvers.com/installation)
- [Pedro Pathing 3 constants](https://pedropathing.com/docs/pathing/tuning/constants) and [OctoQuad localizer](https://pedropathing.com/docs/pathing/tuning/localization/octoquad)
- [BIOBUZZ game and season materials](https://ftc-resources.firstinspires.org/ftc/game)
