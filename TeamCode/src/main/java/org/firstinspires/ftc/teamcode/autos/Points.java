package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

/**
 * Every field point the autonomous routines drive to or bend through, in one place, all
 * tunable in Panels (Configurables, under each routine's name: RedFull, RedLeast).
 *
 * <p>All points are in the team RED field frame: inches, heading in degrees, CCW-positive (see
 * {@link org.firstinspires.ftc.teamcode.utils.Alliance}). The autos read them through
 * {@link BaseAuto#redPose(Point)} when the OpMode is INITialised, so after changing a value in
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

    /** Auto routines/RedFull.pp.json. */
    @Configurable
    public static final class RedFull {
        private RedFull() {
        }

        // ---------- Waypoints ----------
        public static Point START = new Point(52.63535635811941, 6.849936349051341, 270);
        public static Point SHOOT_1 = new Point(54.3307, 14.063, 270);
        public static Point FLOWER_PREP_1 = new Point(19.488, 49.685, 180);
        public static Point FLOWER_1 = new Point(10.7441, 50.1575, 180);
        public static Point SHOOT_2 = new Point(57.943067278180806, 126.01983598981584, 90);
        public static Point FLOWER_PREP_2 = new Point(44.0157, 120.8268, 90);
        public static Point FLOWER_2 = new Point(45.08, 129.5945, 90);
        public static Point SHOOT_2_RETURN = new Point(58.05410766601562, 126.57603672572543, 90);
        public static Point GARDEN_PREP = new Point(21.571446010044642, 5.746956961495535, 180);
        public static Point GARDEN = new Point(7.217008318219866, 5.658124651227679, 180);
        public static Point SHOOT_1_RETURN = new Point(57.67, 15.78, -90);
        public static Point STOP_1 = new Point(9.43345642089844, 96.21348135811941, 0);

        // ---------- Bezier control points ----------
        /** flower1 → shoot2, first control point. */
        public static Point FLOWER_1_TO_SHOOT_2_CTRL_A = new Point(39.1075342942361, 47.42045650357433, 0);
        /** flower1 → shoot2, second control point. */
        public static Point FLOWER_1_TO_SHOOT_2_CTRL_B = new Point(24.04969063199014, 117.33626336084433, 0);
        /** flower2 → shoot2Return. */
        public static Point FLOWER_2_TO_SHOOT_2_CTRL = new Point(48.23847961425781, 107.75913565499441, 0);
        /** shoot2Return → gardenPrep. */
        public static Point SHOOT_2_TO_GARDEN_PREP_CTRL = new Point(25.469588630544294, 125.11931435878847, 0);
        /** shoot1Return → stop1. */
        public static Point SHOOT_1_TO_STOP_1_CTRL = new Point(32.019441483516886, 20.86708279351004, 0);
    }

    /** Auto routines/RedLeast.pp.json. */
    @Configurable
    public static final class RedLeast {
        private RedLeast() {
        }

        // ---------- Waypoints ----------
        /** Original RedLeast.pp.json start; RedLeastAuto now starts at START_2. */
        /** Where RedLeastAuto starts. */
        public static Point START_2 = new Point(66.4173, 134.8425, 90);
        /**
         * Shooting position 2. Starts at the same values as RedFull.SHOOT_2_RETURN (heading 90,
         * like the TeleOp X target) but is its own copy, so tuning one does not move the other.
         * Not in RedLeast.pp.json.
         */
        public static Point SHOOT_2 = new Point(58.05410766601562, 126.57603672572543, 90);
        /** Where RedLeastAuto parks, inside the RED LOADING ZONE (tile A5). */
        public static Point END = new Point(16, 112.52, 180);
    }

    /**
     * BlueLeastAuto's points: the RedLeast points it uses, rotated 180 degrees about the Pedro field
     * centre (70.75, 70.75): blue = (141.5 - x, 141.5 - y, heading + 180). Their own copies, so
     * tuning BLUE in Panels does not move RED (and the other way round).
     */
    @Configurable
    public static final class BlueLeast {
        private BlueLeast() {
        }

        // ---------- Waypoints ----------
        /** Where BlueLeastAuto starts. RedLeast.START_2 rotated. */
        public static Point START_2 = new Point(76.34, 7.8, -90);
        /** Shooting position 2. RedLeast.SHOOT_2 rotated. */
        public static Point SHOOT_2 = new Point(83.445892334, 14.9239632743, -90);
        /** Where BlueLeastAuto parks, inside the BLUE LOADING ZONE. RedLeast.STOP_2 rotated. */
        public static Point STOP_2 = new Point(125.5, 28.98, 0);
    }

    /**
     * BlueFullAuto's points: the RedFull points it uses, rotated 180 degrees about the Pedro field
     * centre (70.75, 70.75): blue = (141.5 - x, 141.5 - y, heading + 180). Their own copies, so
     * tuning BLUE in Panels does not move RED (and the other way round).
     *
     * <p>Headings are written so every turn goes the same way round as on RED (FLOWER_PREP_2 and
     * FLOWER_2 are 270, not -90, so FLOWER_2 → SHOOT_2 turns 270 → 180 like RED's 90 → 0). Keep
     * that in mind when tuning them.
     */
    @Configurable
    public static final class BlueFull {
        private BlueFull() {
        }

        // ---------- Waypoints ----------
        public static Point START = new Point(88.8646436419, 134.6500636509, 90);
        public static Point SHOOT_1 = new Point(85.6693, 128.037, 90);
        public static Point FLOWER_PREP_1 = new Point(122.012, 95.15, 0);
        public static Point FLOWER_1 = new Point(129.5559, 95.0425, 0);
        public static Point SHOOT_2 = new Point(83.5569327218, 15.4801640102, 270);
        public static Point FLOWER_PREP_2 = new Point(97.4843, 20.6732, 270);
        public static Point FLOWER_2 = new Point(96.42, 11.7055, 270);
        public static Point STOP_1 = new Point(132.0665435791, 45.2865186419, 180);

        // ---------- Bezier control points ----------
        /** flower1 → shoot2, first control point. */
        public static Point FLOWER_1_TO_SHOOT_2_CTRL_A = new Point(102.3924657058, 94.0795434964, 0);
        /** flower1 → shoot2, second control point. */
        public static Point FLOWER_1_TO_SHOOT_2_CTRL_B = new Point(117.450309368, 24.1637366392, 0);
    }
}
