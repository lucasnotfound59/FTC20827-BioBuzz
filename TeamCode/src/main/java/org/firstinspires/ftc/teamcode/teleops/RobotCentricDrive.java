package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Robot-centric manual drive: stick-forward moves the way the robot faces. Pose, B/X paths,
 * AUTO handoff and D-pad reset come from {@link PedroDriveTeleOp}, same as Field-Centric.
 */
@TeleOp(name = "Robot-Centric")
public final class RobotCentricDrive extends PedroDriveTeleOp {
    @Override
    protected void driveManually(double forward, double lateral, double turn) {
        follower.manual(forward, lateral, turn);
    }

    @Override
    protected String stickFrameHint() {
        return "Stick-forward follows the robot's facing; the pose still uses the AUTO field frame.";
    }
}
