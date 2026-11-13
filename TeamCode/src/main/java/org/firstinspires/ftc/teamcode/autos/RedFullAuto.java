package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.utils.Alliance;

/**
 * Eleven named segments from Auto routines/RedFull.pp.json, in sequence. Coordinates and headings
 * are in the team RED field frame. The JSON contains no mechanism markers or waits, so
 * shooting and collection actions are attached to its named stops here. After collecting,
 * the intake runs slowly during the return path to retain the POLLEN until shooting.
 */
@Configurable
@Autonomous(name = "Red Full Auto", group = "Red", preselectTeleOp = "Field-Centric")
public final class RedFullAuto extends BaseAuto {
    public static double SHOOT_SECONDS = 2.0;
    public static double FLOWER_SECONDS = 0.5;
    public static double GARDEN_SECONDS = 0.5;
    public static long LONG_PATH_TIMEOUT_MS = 8000;

    public RedFullAuto() {
        super(Alliance.RED);
    }

    @Override
    protected Pose startPose() {
        return redPose(52.63535635811941, 6.849936349051341, 90);
    }

    @Override
    protected Command buildRoutine() {
        Pose start1 = startPose();
        Pose shoot1 = redPose(57.67473929268973, 15.784066336495535, -90);
        Pose flowerPrep1 = redPose(27.75, 46.75, 180);
        Pose flower1 = redPose(11.75, 46.75, 180);
        Pose shoot2 = redPose(57.943067278180806, 126.61983598981584, 0);
        Pose flowerPrep2 = redPose(46.838215419224326, 116.75, 90);
        Pose flower2 = redPose(46.75, 129.75, 90);
        Pose shoot2Return = redPose(58.05410766601562, 126.57603672572543, 90);
        Pose gardenPrep = redPose(21.571446010044642, 5.746956961495535, 180);
        Pose garden = redPose(7.217008318219866, 5.658124651227679, 180);
        Pose shoot1Return = redPose(57.67, 15.78, -90);
        Pose stop1 = redPose(9.43345642089844, 96.21348135811941, 0);

        Path start1ToShoot1 = line(start1, shoot1, Interpolator.constant(Math.toRadians(-90)));
        Path shoot1ToFlowerPrep1 = line(shoot1, flowerPrep1,
                Interpolator.linear(Math.toRadians(-90), Math.toRadians(180)));
        Path flowerPrep1ToFlower1 = line(flowerPrep1, flower1,
                Interpolator.constant(Math.toRadians(180)));
        Path flower1ToShoot2 = Paths.curve(flower1,
                redPose(39.1075342942361, 47.42045650357433, 0),
                redPose(24.04969063199014, 117.33626336084433, 0), shoot2)
                .heading(Interpolator.piecewise()
                        .until(0.3,
                                Interpolator.linear(Math.toRadians(180), Math.toRadians(90)))
                        .until(1.0, Interpolator.constant(0)));
        Path shoot2ToFlowerPrep2 = line(shoot2, flowerPrep2,
                Interpolator.constant(Math.toRadians(90)));
        Path flowerPrep2ToFlower2 = line(flowerPrep2, flower2,
                Interpolator.constant(Math.toRadians(90)));
        Path flower2ToShoot2 = curve(flower2,
                redPose(48.23847961425781, 107.75913565499441, 0), shoot2Return,
                Interpolator.constant(Math.toRadians(90)));
        Path shoot2ToGardenPrep = curve(shoot2Return,
                redPose(25.469588630544294, 125.11931435878847, 0), gardenPrep,
                Interpolator.piecewise()
                        .until(0.7, Interpolator.constant(Math.toRadians(90)))
                        .until(1.0, Interpolator.linear(Math.toRadians(90), Math.toRadians(180))));
        Path gardenPrepToGarden = line(gardenPrep, garden,
                Interpolator.constant(Math.toRadians(180)));
        Path gardenToShoot1 = line(garden, shoot1Return,
                Interpolator.piecewise()
                        .until(0.5, Interpolator.constant(Math.toRadians(180)))
                        .until(1.0,
                                Interpolator.linear(Math.toRadians(180), Math.toRadians(-90))));
        Path shoot1ToStop1 = curve(shoot1Return,
                redPose(32.019441483516886, 20.86708279351004, 0), stop1,
                Interpolator.piecewise()
                        .until(0.7, Interpolator.constant(Math.toRadians(-90)))
                        .until(1.0, Interpolator.constant(0)));

        return new SequentialCommandGroup(
                spinUp(),
                follow(start1ToShoot1),
                shoot(SHOOT_SECONDS),
                follow(shoot1ToFlowerPrep1),
                collectAtFlower(flowerPrep1ToFlower1),
                spinUp(),
                follow(flower1ToShoot2, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS),
                shoot(SHOOT_SECONDS),
                follow(shoot2ToFlowerPrep2),
                collectAtFlower(flowerPrep2ToFlower2),
                spinUp(),
                follow(flower2ToShoot2),
                shoot(SHOOT_SECONDS),
                follow(shoot2ToGardenPrep, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS),
                collectAtGarden(gardenPrepToGarden),
                spinUp(),
                follow(gardenToShoot1, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS),
                shoot(SHOOT_SECONDS),
                follow(shoot1ToStop1, PATH_SPEED_FRACTION, LONG_PATH_TIMEOUT_MS)) {
            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                shooter.stop();
                intake.stop();
                intake.stopSplitter();
            }
        };
    }

    private Command collectAtFlower(Path path) {
        return new SequentialCommandGroup(
                intakeOn(),
                follow(path),
                new InstantCommand(intake::deploySplitter, intake),
                new WaitCommand((long) (FLOWER_SECONDS * 1000))) {
            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                if (interrupted) intake.stop();
                else intake.runIntakeSlow();
                intake.stopSplitter();
            }
        };
    }

    private Command collectAtGarden(Path path) {
        return new SequentialCommandGroup(
                intakeOn(),
                follow(path),
                new WaitCommand((long) (GARDEN_SECONDS * 1000))) {
            @Override
            public void end(boolean interrupted) {
                super.end(interrupted);
                if (interrupted) intake.stop();
                else intake.runIntakeSlow();
            }
        };
    }
}
