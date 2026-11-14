package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.Points.Blue;
import org.firstinspires.ftc.teamcode.utils.Alliance;

/** Drives directly from {@link Points.Blue#START_2} to {@link Points.Blue#STOP_2}. */
@Autonomous(name = "Blue Leave Auto", group = "Blue", preselectTeleOp = "Field-Centric")
public final class BlueLeaveAuto extends BaseAuto {
    public BlueLeaveAuto() {
        super(Alliance.BLUE);
    }

    @Override
    protected Pose startPose() {
        return bluePose(Blue.START_2);
    }

    @Override
    protected Command buildRoutine() {
        return new SequentialCommandGroup(
                // Keep the intake stopped for this drive-only routine.
                intakeOff(),
                follow(line(startPose(), bluePose(Blue.STOP_2))));
    }
}
