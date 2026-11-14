package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.autos.Points.Blue;
import org.firstinspires.ftc.teamcode.utils.Alliance;

/** Waits outside the end zone, then drives to STOP_2 with five seconds of AUTO remaining. */
@Configurable
@Autonomous(name = "Blue Delayed Leave Auto", group = "Blue", preselectTeleOp = "Field-Centric")
public final class BlueDelayedLeaveAuto extends BaseAuto {
    // Remaining match time, not a delay measured from when this command starts.
    public static double MOVE_WHEN_SECONDS_LEFT = 5.0;

    public BlueDelayedLeaveAuto() {
        super(Alliance.BLUE);
    }

    @Override
    protected Pose startPose() {
        return bluePose(Blue.START_OUTSIDE_END_ZONE);
    }

    @Override
    protected Command buildRoutine() {
        return new SequentialCommandGroup(
                intakeOff(),
                new WaitUntilCommand(() -> secondsLeft() <= MOVE_WHEN_SECONDS_LEFT),
                follow(line(startPose(), bluePose(Blue.STOP_2))));
    }
}
