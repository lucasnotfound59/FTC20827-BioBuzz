package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autos.Points.Red;
import org.firstinspires.ftc.teamcode.utils.Alliance;

/** Drives directly from {@link Points.Red#START_2} to {@link Points.Red#STOP_2}. */
@Autonomous(name = "Red Leave Auto", group = "Red", preselectTeleOp = "Field-Centric")
public final class RedLeaveAuto extends BaseAuto {
    public RedLeaveAuto() {
        super(Alliance.RED);
    }

    @Override
    protected Pose startPose() {
        return redPose(Red.START_2);
    }

    @Override
    protected Command buildRoutine() {
        return new SequentialCommandGroup(
                // Keep the intake stopped for this drive-only routine.
                intakeOff(),
                follow(line(startPose(), redPose(Red.STOP_2))));
    }
}
