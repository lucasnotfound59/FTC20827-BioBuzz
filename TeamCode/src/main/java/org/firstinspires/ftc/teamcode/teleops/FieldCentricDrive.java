package org.firstinspires.ftc.teamcode.teleops;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GamepadConfig;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.firstinspires.ftc.teamcode.utils.AutoHandoff;

/** Field-centric manual drive and B/X shooting-position paths through one Pedro follower. */
@Configurable
@TeleOp(name = "Field-Centric")
public final class FieldCentricDrive extends BaseDriveTeleOp {
    // Set these in Panels before INIT. Heading is CCW from field +x, in degrees.
    public static boolean USE_AUTO_HANDOFF = true;
    public static boolean START_ON_BLUE = false;
    public static double START_X_IN = 70.75;
    public static double START_Y_IN = 70.75;
    public static double START_HEADING_DEG = 0.0;

    // Shooting poses from RedFullAuto, in the RED field frame.
    private static final Pose SHOOT1 = new Pose(58, 14,
            Math.toRadians(-90));
    private static final Pose SHOOT2 = new Pose(57.8, 125.2, Math.toRadians(90));

    private Follower follower;
    private GamepadConfig controls;
    private Alliance alliance;
    private boolean usingHandoff;
    private boolean shoot1WasHeld, shoot2WasHeld;

    @Override
    protected void initializeDrive(GamepadConfig controls) {
        this.controls = controls;
        AutoHandoff handoff = AutoHandoff.take();
        usingHandoff = USE_AUTO_HANDOFF && handoff != null;
        alliance = usingHandoff ? handoff.alliance : (START_ON_BLUE ? Alliance.BLUE : Alliance.RED);
        Pose startPose = usingHandoff ? handoff.pose
                : new Pose(START_X_IN, START_Y_IN, Math.toRadians(START_HEADING_DEG));

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.holdEnd.set(false);
    }

    @Override
    protected void updateDriveInitialization() {
        follower.localizer.update();
        telemetry.addData("Start pose source", usingHandoff ? "AUTO" : "configured");
        telemetry.addLine("B: shoot1. X: shoot2. Move a drive stick to cancel.");
        telemetry.addLine("Stick-forward follows field +x; Auto and TeleOp share the same pose frame.");
    }

    @Override
    protected void beforeDriveRun() {
        double forward = applyDeadband(controls.getForward()) * GamepadConfig.DRIVE_LIMIT;
        // Pedro uses left-positive strafe and CCW-positive turn.
        double lateral = -applyDeadband(controls.getStrafe()) * GamepadConfig.DRIVE_LIMIT;
        double turn = -applyDeadband(controls.getTurn()) * GamepadConfig.TURN_LIMIT;
        boolean driverMoving = forward != 0 || lateral != 0 || turn != 0;
        boolean shoot1Held = controls.isDriveToShoot1Held();
        boolean shoot2Held = controls.isDriveToShoot2Held();

        if (!driverMoving) {
            if (shoot1Held && !shoot1WasHeld) followTo(SHOOT1);
            else if (shoot2Held && !shoot2WasHeld) followTo(SHOOT2);
        }
        shoot1WasHeld = shoot1Held;
        shoot2WasHeld = shoot2Held;

        if (driverMoving || follower.mode() != Follower.Mode.FOLLOW) {
            follower.manual(ManualDrive.fieldCentric(forward, lateral, turn,
                    follower.pose().heading()));
        }
        follower.update();
    }

    private void followTo(Pose redTarget) {
        Pose start = follower.pose();
        Pose target = alliance == Alliance.RED ? redTarget : Alliance.RED.fromBlue(redTarget);
        follower.follow(Paths.line(start, target)
                .heading(Interpolator.linear(start.heading(), target.heading())));
    }

    private double applyDeadband(double input) {
        double deadband = GamepadConfig.DRIVE_DEADBAND;
        if (Math.abs(input) <= deadband) return 0;
        return Math.copySign((Math.abs(input) - deadband) / (1 - deadband), input);
    }

    @Override
    protected void stopDrive() {
        follower.stop();
        follower.drivetrain.stop();
    }

    @Override
    protected void addDriveTelemetry() {
        Pose pose = follower.pose();
        telemetry.addData("Pedro", follower.mode());
        telemetry.addData("Field x (in)", pose.x());
        telemetry.addData("Field y (in)", pose.y());
        telemetry.addData("Field heading (deg)", Math.toDegrees(pose.heading()));
    }
}
