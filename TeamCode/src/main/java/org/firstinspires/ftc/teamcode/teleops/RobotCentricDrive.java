package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.GamepadConfig;

/** Mecanum drive with translation relative to the robot's current facing. */
@TeleOp(name = "Robot-Centric")
public final class RobotCentricDrive extends BaseDriveTeleOp {
    private Drive drive;

    @Override
    protected void initializeDrive(GamepadConfig controls) {
        drive = new Drive(hardwareMap);
        drive.setDefaultCommand(new DriveCommand(
                drive,
                controls::getForward,
                controls::getStrafe,
                controls::getTurn,
                GamepadConfig.DRIVE_DEADBAND,
                GamepadConfig.DRIVE_LIMIT,
                GamepadConfig.TURN_LIMIT));
    }

    @Override
    protected void stopDrive() {
        drive.stop();
    }

    @Override
    protected void addDriveTelemetry() {
        telemetry.addData("Drive", "Robot reference");
    }
}
