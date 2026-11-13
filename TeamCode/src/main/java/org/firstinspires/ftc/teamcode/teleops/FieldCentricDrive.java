package org.firstinspires.ftc.teamcode.teleops;

import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Field-centric manual drive: stick-forward always moves along field +x, whatever the robot's
 * facing. Pose, B/X paths, AUTO handoff and D-pad reset come from {@link PedroDriveTeleOp}.
 */
@TeleOp(name = "Field-Centric")
public final class FieldCentricDrive extends PedroDriveTeleOp {
    @Override
    protected void driveManually(double forward, double lateral, double turn) {
        follower.manual(ManualDrive.fieldCentric(forward, lateral, turn,
                follower.pose().heading()));
    }

    @Override
    protected String stickFrameHint() {
        return "Stick-forward follows field +x; Auto and TeleOp share the same pose frame.";
    }
}
