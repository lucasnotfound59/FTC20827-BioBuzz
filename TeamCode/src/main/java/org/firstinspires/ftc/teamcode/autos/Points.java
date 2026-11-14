package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/**
 * Every field point the autonomous routines drive to or bend through, in one place, all
 * shared by alliance and tunable in Panels (Configurables, under Red and Blue).
 *
 * <p>Points use field coordinates for their named alliance: inches, heading in degrees,
 * CCW-positive (see {@link org.firstinspires.ftc.teamcode.utils.Alliance}). Red and Blue are
 * independently tunable. All autos and TeleOp share the same points for each alliance.
 * The autos read them through {@link BaseAuto#redPose(Point)} or
 * {@link BaseAuto#bluePose(Point)} when the OpMode is INITialised, so after changing a value in
 * Panels, INIT the auto again. Changes made in Panels are lost on a robot restart: copy good
 * values back into this file.
 *
 * <p>Waypoints are where the robot stops; control points only bend a Bezier curve, so their
 * heading is unused and left at 0.
 */
public final class Points {
    private Points() {
    }

    /** A field point Panels can edit (unlike Pedro's immutable {@link Pose}). */
    public static final class Point {
        public double x;
        public double y;
        public double headingDeg;

        public Point(double x, double y, double headingDeg) {
            this.x = x;
            this.y = y;
            this.headingDeg = headingDeg;
        }

        /** The current values as a Pedro pose (heading in radians). */
        public Pose toPose() {
            return new Pose(x, y, Math.toRadians(headingDeg));
        }
    }

    /** Shared RED alliance points for autonomous routines and TeleOp. */
    @Configurable
    public static final class Red {
        private Red() {
        }

        // ---------- Waypoints ----------
        public static Point START = new Point(49.8, 8.34, 270);
        /** Starting location for the Least routine. */
        public static Point START_2 = new Point(63.46, 132.8, 90);
        public static Point SHOOT_1 = new Point(54.5, 16.5, 270);
        public static Point FLOWER_PREP_1 = new Point(16.45, 48.54, 180);
        public static Point FLOWER_1 = new Point(9.5, 48.54, 180);
        public static Point SHOOT_2 = new Point(55.4, 124.4, 90);
        public static Point FLOWER_PREP_2 = new Point(44.0157, 120.8268, 90);
        public static Point FLOWER_2 = new Point(45.08, 129.5945, 90);
        public static Point SHOOT_2_RETURN = new Point(58.05410766601562, 126.57603672572543, 90);
        public static Point GARDEN_PREP = new Point(20.47, 6.145, 180);
        public static Point GARDEN = new Point(4.96, 6.145, 180);
        public static Point SHOOT_1_RETURN = new Point(57.67, 15.78, -90);
        public static Point STOP_1 = new Point(9.43345642089844, 96.21348135811941, 0);
        /** Parking location inside the RED LOADING ZONE. */
        public static Point STOP_2 = new Point(11, 118.66, 0);

        // ---------- Bezier control points ----------
        /** start2 → shoot2, from RedCooperateUp.pp.json. */
        public static Point START_2_TO_SHOOT_2_CTRL = new Point(26.7085462999709, 123.37946316254681, 0);
        /** flower1 → shoot2, first control point. */
        public static Point FLOWER_1_TO_SHOOT_2_CTRL_A = new Point(39.1075342942361, 47.42045650357433, 0);
        /** flower1 → shoot2, second control point. */
        public static Point FLOWER_1_TO_SHOOT_2_CTRL_B = new Point(24.04969063199014, 117.33626336084433, 0);
        /** flower2 → shoot2Return. */
        public static Point FLOWER_2_TO_SHOOT_2_CTRL = new Point(48.23847961425781, 107.75913565499441, 0);
        /** shoot1 → gardenPrep, from RedCooperationDown.pp.json. */
        public static Point SHOOT_1_TO_GARDEN_PREP_CTRL = new Point(55.464719330997355, 24.795270819566625, 0);
        /** shoot2Return → gardenPrep. */
        public static Point SHOOT_2_TO_GARDEN_PREP_CTRL = new Point(25.469588630544294, 125.11931435878847, 0);
        /** shoot1Return → stop1. */
        public static Point SHOOT_1_TO_STOP_1_CTRL = new Point(32.019441483516886, 20.86708279351004, 0);
    }

    /**
     * Shared BLUE alliance points for autonomous routines and TeleOp. Based on RED points
     * rotated 180 degrees about the field centre (70.75, 70.75), then tuned independently.
     */
    @Configurable
    public static final class Blue {
        private Blue() {
        }

        // ---------- Waypoints ----------
        public static Point START = new Point(86.8,132.5, 90);
        /** Delayed parking start outside the end zone; tune for the actual placement before INIT. */
        public static Point START_OUTSIDE_END_ZONE = new Point(129.5, 53.6, 180);
        /** Starting location for the Least routine. */
        public static Point START_2 = new Point(73.5, 7.8, -90);
        public static Point SHOOT_1 = new Point(81, 126, 90);
        public static Point FLOWER_PREP_1 = new Point(120,93.7, 0);
        public static Point FLOWER_1 = new Point(127.5,93.7, 0);
        public static Point SHOOT_2 = new Point(81.5, 15.6, 270);
        public static Point FLOWER_PREP_2 = new Point(92, 17.75, 270);
        public static Point FLOWER_2 = new Point(91.5, 11.2, 270);
        /** Garden approach and collection points, rotated from RED. */
        public static Point GARDEN_PREP = new Point(119, 134, 0);
        public static Point GARDEN = new Point(132.7, 134,  0);
        public static Point STOP_1 = new Point(126.5, 45, 180);
        /** Parking location inside the BLUE LOADING ZONE. */
        public static Point STOP_2 = new Point(125.5, 21, 180);

        // ---------- Bezier control points ----------
        /** start2 → shoot2, RedCooperateUp.pp.json rotated to BLUE. */
        public static Point START_2_TO_SHOOT_2_CTRL = new Point(114.7914537000291, 18.12053683745319, 0);
        /** flower2 → shoot2, RedCooperateUp.pp.json rotated to BLUE. */
        public static Point FLOWER_2_TO_SHOOT_2_CTRL = new Point(96.42203302765787, 26.95597458960405, 0);
        /** shoot1 → gardenPrep, rotated from RED. */
        public static Point SHOOT_1_TO_GARDEN_PREP_CTRL = new Point(86.03528066900265, 116.70472918043338, 0);
        /** shoot1 → stop1, rotated from RED. */
        public static Point SHOOT_1_TO_STOP_1_CTRL = new Point(109.48055851648311, 120.63291720648996, 0);
        /** flower1 → shoot2, first control point. */
        public static Point FLOWER_1_TO_SHOOT_2_CTRL_A = new Point(102.3924657058, 94.0795434964, 0);
        /** flower1 → shoot2, second control point. */
        public static Point FLOWER_1_TO_SHOOT_2_CTRL_B = new Point(117.450309368, 24.1637366392, 0);
    }
}
