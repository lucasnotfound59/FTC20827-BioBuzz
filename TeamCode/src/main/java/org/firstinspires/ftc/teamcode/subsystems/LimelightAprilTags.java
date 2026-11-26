package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.jetbrains.annotations.NotNull;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * AprilTag detection services on the Limelight that is bolted to the turret: which CELL of the
 * HIVE faces the camera, which CELL the HIVE is taking down, and how far the turret still has
 * to turn.
 *
 * <p><b>What the field gives us</b> (BIOBUZZ Competition Manual TU06, sections 9.6 and 9.9):
 * one frame at the centre of the FIELD holds a red and a blue HIVE; each HIVE is a bi-stable
 * seesaw of two CELLS approximately 18.8 in (47.8 cm) apart whose pivot axis sits 43.95 in
 * (111.65 cm) above the TILES, so the two CELLS lie on the AUDIENCE_SIDE to JUDGE_SIDE line and
 * the pivot axis is parallel to the red-blue axis. Each CELL carries a 4-tag cluster of 3.25 in
 * (8.25 cm) 36h11 tags on its bottom face, facing downward towards the TILES. That is why the
 * UP CELL's tags are visible from a shooting pose and the DOWN CELL's are not, and why a tip is
 * a rotation about a <em>horizontal</em> axis, never about the vertical.
 *
 * <p><b>Why the tilt test is a rotation, not an image drift.</b> For a tag normal {@code n} and a
 * HIVE rotation {@code t} about the pivot axis {@code a}, the angle seen between the old and the
 * new normal is
 * <pre>
 * cos(phi) = cos(t) + (n . a)^2 (1 - cos(t))
 * </pre>
 * so it equals the tip angle {@code t} exactly when {@code n} is perpendicular to {@code a},
 * which is the design case, and falls off only as fast as {@code (n . a)^2} allows: at
 * {@code n . a = 0.3} a 12 deg tip reads 11.5 deg, but at {@code n . a = 0.8}, which a 20 deg
 * turret angle already produces, it reads 7.0 deg - below the threshold, a missed tip. Two
 * consequences matter. First, the measurement is a relative rotation between the baseline and the
 * current frame, so an oblique view, a shifted crop or a rolled robot do not bias it - only their
 * effect on pose noise does. Second, a rotating turret also shows up as a rotation, but about the
 * vertical axis only, so removing the vertical component separates the two: turret motion is
 * discarded, HIVE motion is kept, and no turret angle is needed to detect a tip.
 *
 * <p><b>Where the vertical is, without knowing the turret angle.</b> The optical axis is pitched
 * up by {@link #CAMERA_PITCH_DEG} and the mount has no roll, so in Limelight camera coordinates
 * (x right, y down, z forward) the world vertical is
 * <pre>
 * v = (0, -cos(pitch), sin(pitch))
 * </pre>
 * which has no yaw term: turning the turret about that same vertical leaves {@code v} unchanged,
 * so the decomposition holds at every turret angle. A pitch error {@code dp} leaks a turret
 * rotation {@code p} into the tilt as {@code p sin(dp)}: a 10 deg turret swing with a 5 deg pitch
 * error leaks 0.9 deg, far below {@link #TILT_CONFIRM_DEG}.
 *
 * <p><b>Naming the descending CELL.</b> The pivot axis is the field's red-blue axis (team-frame
 * +x), and +y points away from the audience, i.e. towards the JUDGE_SIDE. A right-handed rotation
 * about +x lifts the +y CELL, so the sign of the measured rotation about +x names the descending
 * CELL once the camera yaw in the field frame is known: a positive projection takes the
 * AUDIENCE_SIDE CELL down, a negative one the JUDGE_SIDE CELL. A tip always takes the UP CELL
 * down, so a direction that disagrees with the observed UP CELL is a measurement fault and is
 * reported as UNKNOWN. {@link #setCameraYawFieldDeg} supplies that yaw from the robot heading
 * plus the turret angle; without it a tip is still detected, but stays UNDIRECTED.
 *
 * <p><b>Services.</b> They never own hardware other than the camera and never decide when to
 * shoot:
 * <ul>
 * <li><b>CELL state</b> ({@link #getCellPosition}, {@link #getUpCell}): the crop keeps the upper
 *     half of the frame, where roughly half of the UP CELL's tags are detected; the DOWN CELL's
 *     tags are not detected at all.</li>
 * <li><b>Tilt</b> ({@link #startTiltObservation} right after the shot, {@link
 *     #updateTiltPrediction} in the loop, {@link #getTiltDirection}): reports TILTING as soon as
 *     the fall is visible instead of waiting out the ~3 s at the end of AUTO, and names the
 *     descending CELL so a routine can tell a partner's tip from its own and stop feeding a CELL
 *     that has left the shooting pose.</li>
 * <li><b>Turret aim assist</b> ({@link #getAimYawErrorDeg}, {@link #isAimedAt}): the camera rides
 *     the turret, so {@code tx} is already the turret error, not a robot-relative bearing.</li>
 * </ul>
 *
 * <p>Every consumer passes one gate: connected, valid, younger than {@link #MAX_RESULT_AGE_MS},
 * and reporting the AprilTag pipeline. {@link Limelight3A#pipelineSwitch} is fire-and-forget, so
 * the pipeline index is read back from the result. Camera pose and lens calibration live on the
 * Limelight; {@link #setRobotHeadingDeg} feeds heading for field-space output, and no MegaTag 2
 * pose is consumed here. Because the camera turns with the turret, the Limelight's configured
 * robot-space frame is only meaningful at one turret angle; this class therefore works in camera
 * space and uses the measured {@link #CAMERA_PITCH_DEG} instead of that transform.
 */
@Configurable
public final class LimelightAprilTags extends SubsystemBase {
    // Pipeline ID must match the configuration at limelight.local:5801.
    public static String LIMELIGHT_NAME = "limelight";
    public static int PIPELINE_APRILTAG = 0;
    public static int POLL_RATE_HZ = 100;

    // A result older than this cannot be used for a control decision: a stale frame reports the
    // previous pose, which is exactly what a tilt or an aim correction must not act on.
    public static long MAX_RESULT_AGE_MS = 250;

    // With the pipeline cropped to normalized Y in [-0.25, 1], a detected tag at or above this
    // line counts as upper-half evidence of the UP CELL. Measured on the robot: the bottom row
    // of the UP CELL lands just above it, and the DOWN CELL's tags never appear at all.
    public static double UP_CELL_MIN_Y = 0.0;

    // A tag at least this much of the image is worth trusting; smaller targets are noise.
    public static double TAG_MIN_AREA = 0.1;
    // One CELL carries this many tags, so more visible tags cannot all be on one UP CELL.
    public static int MAX_TAGS_AT_SHOOTING_POS = Alliance.TAGS_PER_CELL;

    // ---------- Geometry, TU06 sections 9.6 and 9.9 ----------
    // Used for the tilt derivation and the range diagnostics: the clusters are 3.25 in square
    // 36h11 tags, one cluster on the bottom face of each CELL, CELLS 18.8 in apart on a pivot
    // 43.95 in above the TILES.
    public static double TAG_SIZE_IN = 3.25;
    public static double HIVE_CELL_SPACING_IN = 18.8;
    public static double HIVE_PIVOT_HEIGHT_IN = 43.95;

    // Mount measurements the decomposition needs, in degrees and inches. Measure both on the
    // robot: pitch is the optical axis above horizontal, height is the lens above the TILES.
    public static double CAMERA_PITCH_DEG = 30.0;
    public static double CAMERA_HEIGHT_IN = 12.0;

    // Tip angle, as the horizontal component of the tag's relative rotation, that proves a tip.
    public static double TILT_CONFIRM_DEG = 10.0;
    // A single sample that claims more than this cannot be a HIVE tip; discard it as a bad pose.
    public static double TILT_MAX_STEP_DEG = 60.0;
    // How long to watch for a tip. A real tip reaches the threshold in well under a second, so
    // the window mostly bounds the wait when the HIVE is not tipping.
    public static long TILT_WINDOW_MS = 2500;
    // A tag can drop out for a frame; once the HIVE falls, its tags stay gone.
    public static long TAG_LOST_MS = 100;

    // Aim assist: while the error is inside this band the turret counts as on target.
    public static double AIM_TOLERANCE_DEG = 0.75;

    /**
     * One of the two CELLS of a HIVE, named for the side of the FIELD it sits on: the
     * AUDIENCE_SIDE faces the stands, the JUDGE_SIDE faces the FIELD STAFF, which the manual
     * calls "the side of the FIELD opposite of the audience" (TU06 9.9).
     */
    public enum Cell { AUDIENCE_SIDE, JUDGE_SIDE }

    /** Where one of our CELLs sits, judged from the tags the camera can see. */
    public enum CellPosition {
        /** Upper-half tags of this CELL are visible, so it is the UP CELL and can be shot. */
        UP,
        /** The other CELL is the UP CELL, so this one is DOWN. */
        DOWN,
        /** More tags are visible than one UP CELL carries: not at a shooting pose. */
        NOT_AT_SHOOTING_POSITION,
        /** Both CELLs are visible, no CELL is visible, or there is no usable result. */
        UNKNOWN
    }

    /** What a tilt observation has concluded. */
    public enum TiltPrediction {
        /** Watching; the window is still open. */
        OBSERVING,
        /** The HIVE is tipping. */
        TILTING,
        /** The window closed with the tags still in place, so the HIVE is not tipping. */
        NOT_TILTING,
        /** Nothing could be watched: no tag of the UP CELL, or no usable result. */
        UNKNOWN;

        /** True once there is an answer, so polling can stop. */
        public boolean isTerminal() {
            return this != OBSERVING;
        }
    }

    /** Which CELL the HIVE is taking down. */
    public enum TiltDirection {
        /** A tip was confirmed, but the camera yaw is unknown or the sign is inconsistent. */
        UNKNOWN,
        /** No tilt was observed through the window. */
        NONE,
        /** The AUDIENCE_SIDE CELL is descending, so the JUDGE_SIDE CELL becomes the UP CELL. */
        AUDIENCE_SIDE,
        /** The JUDGE_SIDE CELL is descending, so the AUDIENCE_SIDE CELL becomes the UP CELL. */
        JUDGE_SIDE
    }

    private final Limelight3A limelight;
    private LLResult result;

    // What a tilt observation is watching, captured when it starts: the camera-space orientation
    // and the normalized image centre of each upper-half tag of the UP CELL.
    private final Map<Integer, double[][]> baselineRotations = new HashMap<>();
    private final Map<Integer, double[]> baselineCenters = new HashMap<>();
    private final ElapsedTime sinceObservationStart = new ElapsedTime();
    private final ElapsedTime sinceTagsSeen = new ElapsedTime();
    private TiltPrediction tiltPrediction = TiltPrediction.UNKNOWN;
    private TiltDirection tiltDirection = TiltDirection.UNKNOWN;
    private Cell watchedCell;
    private double tiltAngleDeg;
    private double turretComponentDeg;
    private double cameraYawFieldDeg = Double.NaN;
    private boolean turretLocked;

    public LimelightAprilTags(@NotNull final HardwareMap hMap) {
        limelight = hMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.setPollRateHz(POLL_RATE_HZ);
        limelight.start();
        // Fire-and-forget by design; freshResult() verifies the reported pipeline instead.
        limelight.pipelineSwitch(PIPELINE_APRILTAG);
        result = limelight.getLatestResult();
    }

    @Override
    public void periodic() {
        result = limelight.getLatestResult();
    }

    /** Current heading in degrees for field-space output; call once per loop in AUTO/TELEOP. */
    public void setRobotHeadingDeg(final double headingDeg) {
        limelight.updateRobotOrientation(headingDeg);
    }

    /**
     * The camera's optical-axis yaw in the team field frame, in degrees counter-clockwise from
     * +x: robot heading plus turret angle plus any mount offset. Supplying it lets a confirmed
     * tip be named as a CELL; without it a tip is still detected but stays UNDIRECTED.
     */
    public void setCameraYawFieldDeg(final double cameraYawFieldDeg) {
        this.cameraYawFieldDeg = cameraYawFieldDeg;
    }

    /**
     * True while the turret is holding still. Only the "tags disappeared" fallback needs it: a
     * moving turret can lose the tags without the HIVE moving, so that path stays off until the
     * caller says the mount is locked.
     */
    public void setTurretLocked(final boolean turretLocked) {
        this.turretLocked = turretLocked;
    }

    // ---------- Result validity and freshness ----------

    /**
     * The latest result if it can be used right now, else null: connected, valid, inside the age
     * policy, and reporting the AprilTag pipeline.
     */
    private LLResult freshResult() {
        if (!limelight.isConnected()) {
            return null;
        }
        final LLResult latest = result;
        if (latest == null || !latest.isValid()) {
            return null;
        }
        if (latest.getStaleness() > MAX_RESULT_AGE_MS) {
            return null;
        }
        if (latest.getPipelineIndex() != PIPELINE_APRILTAG) {
            return null;
        }
        return latest;
    }

    /** Age of the latest frame in ms, or Long.MAX_VALUE before the first frame. */
    public long getResultAgeMs() {
        return result == null ? Long.MAX_VALUE : result.getStaleness();
    }

    /** True once the camera reports the AprilTag pipeline, not merely after pipelineSwitch(). */
    public boolean isPipelineSynced() {
        return result != null && result.getPipelineIndex() == PIPELINE_APRILTAG;
    }

    // ---------- HIVE CELL state ----------

    /** Where our CELL on {@code side} sits, judged from the upper-half tags of the crop. */
    public CellPosition getCellPosition(@NotNull final Alliance alliance,
                                        @NotNull final Cell side) {
        final LLResult live = freshResult();
        if (live == null) {
            return CellPosition.UNKNOWN;
        }
        final int audience = countUpCellTags(live, alliance, Cell.AUDIENCE_SIDE);
        final int judge = countUpCellTags(live, alliance, Cell.JUDGE_SIDE);
        final int total = audience + judge;
        if (total > MAX_TAGS_AT_SHOOTING_POS) {
            return CellPosition.NOT_AT_SHOOTING_POSITION;
        }
        if (total == 0) {
            // No CELL tag at all means "not looking at a shooting pose", not "our CELL is DOWN":
            // the DOWN CELL's tags are invisible too, so DOWN cannot be claimed here.
            return CellPosition.UNKNOWN;
        }
        if (audience > 0 && judge > 0) {
            return CellPosition.UNKNOWN;
        }
        final Cell up = audience > 0 ? Cell.AUDIENCE_SIDE : Cell.JUDGE_SIDE;
        return side == up ? CellPosition.UP : CellPosition.DOWN;
    }

    /** The CELL that is UP, or null if neither is or the tags cannot tell. */
    public Cell getUpCell(@NotNull final Alliance alliance) {
        final boolean audienceUp = getCellPosition(alliance, Cell.AUDIENCE_SIDE) == CellPosition.UP;
        final boolean judgeUp = getCellPosition(alliance, Cell.JUDGE_SIDE) == CellPosition.UP;
        if (audienceUp == judgeUp) {
            return null;
        }
        return audienceUp ? Cell.AUDIENCE_SIDE : Cell.JUDGE_SIDE;
    }

    /** Upper-half tags of one CELL in the latest usable result. */
    public int getVisibleTagCount(@NotNull final Alliance alliance, @NotNull final Cell side) {
        final LLResult live = freshResult();
        return live == null ? 0 : countUpCellTags(live, alliance, side);
    }

    /** All of one alliance's CELL tags in the latest usable result, upper half or not. */
    public int getAllianceTagCount(@NotNull final Alliance alliance) {
        final LLResult live = freshResult();
        if (live == null) {
            return 0;
        }
        int count = 0;
        for (LLResultTypes.FiducialResult tag : live.getFiducialResults()) {
            if (tag.getTargetArea() >= TAG_MIN_AREA && alliance.ownsTag(tag.getFiducialId())) {
                count++;
            }
        }
        return count;
    }

    // ---------- Tilt prediction and direction ----------

    /**
     * Starts watching the UP CELL's upper-half tags. Call it right after the shot, from a held
     * pose, then poll {@link #updateTiltPrediction()}. Reports UNKNOWN straight away when there
     * is no usable result or no tag of that CELL to watch.
     */
    public void startTiltObservation(@NotNull final Alliance alliance,
                                     @NotNull final Cell upSide) {
        baselineRotations.clear();
        baselineCenters.clear();
        watchedCell = upSide;
        tiltAngleDeg = 0.0;
        turretComponentDeg = 0.0;
        tiltPrediction = TiltPrediction.UNKNOWN;
        tiltDirection = TiltDirection.UNKNOWN;
        final LLResult live = freshResult();
        if (live == null) {
            return;
        }
        for (LLResultTypes.FiducialResult tag : live.getFiducialResults()) {
            if (tag.getTargetArea() < TAG_MIN_AREA) {
                continue;
            }
            if (cellOfTag(alliance, tag.getFiducialId()) != upSide) {
                continue;
            }
            final double[] center = tagCenter(tag);
            if (center == null || center[1] < UP_CELL_MIN_Y) {
                // Outside the upper half of the crop: not one of the bottom tags that prove a tip.
                continue;
            }
            final double[][] rotation = tagRotation(tag);
            if (rotation == null) {
                continue;
            }
            baselineRotations.put(tag.getFiducialId(), rotation);
            baselineCenters.put(tag.getFiducialId(), center);
        }
        if (baselineRotations.isEmpty()) {
            return;
        }
        sinceObservationStart.reset();
        sinceTagsSeen.reset();
        tiltPrediction = TiltPrediction.OBSERVING;
    }

    /**
     * Polls the tilt prediction; call once per loop while it is not terminal. Returns OBSERVING
     * until the watched tags confirm or rule out a tip, and UNKNOWN if the frame becomes unusable
     * before anything could be concluded.
     */
    public TiltPrediction updateTiltPrediction() {
        if (tiltPrediction.isTerminal()) {
            return tiltPrediction;
        }
        if (!limelight.isConnected()) {
            return tiltPrediction = TiltPrediction.UNKNOWN;
        }
        final LLResult live = freshResult();
        if (live == null) {
            // Stale, invalid, or still on the old pipeline: no new information. A stale frame
            // must not confirm a tip, so this only ever ends as UNKNOWN.
            return sinceObservationStart.milliseconds() >= TILT_WINDOW_MS
                    ? tiltPrediction = TiltPrediction.UNKNOWN
                    : TiltPrediction.OBSERVING;
        }

        // Mean relative rotation over the watched tags, in camera coordinates and degrees.
        final double[] cameraUp = cameraUp();
        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;
        int visible = 0;
        double largestStep = 0.0;
        for (LLResultTypes.FiducialResult tag : live.getFiducialResults()) {
            final int id = tag.getFiducialId();
            final double[][] baseline = baselineRotations.get(id);
            if (baseline == null || tag.getTargetArea() < TAG_MIN_AREA) {
                continue;
            }
            final double[][] current = tagRotation(tag);
            if (current == null) {
                continue;
            }
            final double[] step = rotationVector(relativeRotation(current, baseline));
            sumX += step[0];
            sumY += step[1];
            sumZ += step[2];
            largestStep = Math.max(largestStep, norm(step));
            visible++;
        }

        if (visible > 0) {
            sinceTagsSeen.reset();
        }

        if (largestStep > TILT_MAX_STEP_DEG) {
            // A jump this large is a bad pose or a different tag, not a tip: refuse to decide.
            return tiltPrediction = TiltPrediction.UNKNOWN;
        }

        if (visible > 0) {
            final double[] mean = {sumX / visible, sumY / visible, sumZ / visible};
            final double alongUp = dot(mean, cameraUp);
            final double[] horizontal = {
                    mean[0] - alongUp * cameraUp[0],
                    mean[1] - alongUp * cameraUp[1],
                    mean[2] - alongUp * cameraUp[2]
            };
            turretComponentDeg = alongUp;
            tiltAngleDeg = norm(horizontal);
            if (tiltAngleDeg >= TILT_CONFIRM_DEG) {
                tiltDirection = directionOf(horizontal);
                return tiltPrediction = TiltPrediction.TILTING;
            }
        }

        if (visible == 0 && turretLocked && sinceTagsSeen.milliseconds() >= TAG_LOST_MS) {
            // With a locked mount nothing but the HIVE falling can keep the UP CELL's tags out of
            // view, so their loss is a tilt signal; naming the direction needs a rotation, which
            // this path does not have.
            tiltDirection = TiltDirection.UNKNOWN;
            return tiltPrediction = TiltPrediction.TILTING;
        }
        if (sinceObservationStart.milliseconds() >= TILT_WINDOW_MS) {
            tiltDirection = TiltDirection.NONE;
            return tiltPrediction = TiltPrediction.NOT_TILTING;
        }
        return TiltPrediction.OBSERVING;
    }

    // ---------- Turret aim assist ----------

    /**
     * True when there is a usable result with one of this alliance's tags to aim at, so an AUTO
     * or TELEOP aim assist can act on {@link #getAimYawErrorDeg}.
     */
    public boolean hasAimTarget(@NotNull final Alliance alliance) {
        return selectAimTag(alliance) != null;
    }

    /**
     * Signed yaw error, in degrees, that centres the aim target: positive means the turret has to
     * rotate counter-clockwise (robot's left), negative clockwise. The camera rides the turret, so
     * this is directly the turret error and the turret's zero is where the camera looks straight
     * ahead; this class never commands a motor. NaN when there is no usable result or no tag to
     * aim at, so a caller must check {@link #hasAimTarget} first.
     */
    public double getAimYawErrorDeg(@NotNull final Alliance alliance) {
        final LLResultTypes.FiducialResult tag = selectAimTag(alliance);
        return tag == null ? Double.NaN : -tag.getTargetXDegrees();
    }

    /** Yaw error toward one CELL in particular, for a routine that knows which CELL it wants. */
    public double getAimYawErrorDeg(@NotNull final Alliance alliance, @NotNull final Cell side) {
        final LLResultTypes.FiducialResult tag = selectAimTag(alliance, side);
        return tag == null ? Double.NaN : -tag.getTargetXDegrees();
    }

    /** Elevation of the aim target in degrees, positive up; NaN without a usable result. */
    public double getAimPitchDeg(@NotNull final Alliance alliance) {
        final LLResultTypes.FiducialResult tag = selectAimTag(alliance);
        return tag == null ? Double.NaN : tag.getTargetYDegrees();
    }

    /** True while the aim yaw error is inside {@link #AIM_TOLERANCE_DEG}. */
    public boolean isAimedAt(@NotNull final Alliance alliance) {
        final double error = getAimYawErrorDeg(alliance);
        return !Double.isNaN(error) && Math.abs(error) <= AIM_TOLERANCE_DEG;
    }

    /**
     * Straight-line distance to the aim target in metres, for a hood or flywheel assist; NaN
     * without a target. Convert before comparing it with a Pedro pose, which is in inches.
     */
    public double getAimRange(@NotNull final Alliance alliance) {
        final LLResultTypes.FiducialResult tag = selectAimTag(alliance);
        if (tag == null) {
            return Double.NaN;
        }
        final Pose3D pose = tag.getTargetPoseCameraSpace();
        if (pose == null) {
            return Double.NaN;
        }
        // Pose3D positions carry their own unit; normalise before combining with anything.
        final Position position = pose.getPosition().toUnit(DistanceUnit.METER);
        return Math.sqrt(position.x * position.x
                + position.y * position.y
                + position.z * position.z);
    }

    // ---------- Diagnostics ----------

    public TiltPrediction getTiltPrediction() {
        return tiltPrediction;
    }

    /** Which CELL the HIVE is taking down; NONE when the window ruled a tip out. */
    public TiltDirection getTiltDirection() {
        return tiltDirection;
    }

    /** Measured tip angle in degrees: the horizontal part of the watched tags' rotation. */
    public double getTiltAngleDeg() {
        return tiltAngleDeg;
    }

    /**
     * Vertical part of the watched rotation in degrees, seen by the camera: the negative of the
     * turret's own turn, because the tag moves the other way relative to a turning camera. A
     * locked mount keeps this near zero, so a large value means the turret is turning, or that the
     * baseline was taken while it turned.
     */
    public double getTurretComponentDeg() {
        return turretComponentDeg;
    }

    /** Tags being watched, or 0 when the last observation could not start. */
    public int getBaselineTagCount() {
        return baselineRotations.size();
    }

    // ---------- Internal ----------

    private int countUpCellTags(@NotNull final LLResult live, @NotNull final Alliance alliance,
                                @NotNull final Cell side) {
        int count = 0;
        for (LLResultTypes.FiducialResult tag : live.getFiducialResults()) {
            if (tag.getTargetArea() < TAG_MIN_AREA) {
                continue;
            }
            if (cellOfTag(alliance, tag.getFiducialId()) != side) {
                continue;
            }
            final double[] center = tagCenter(tag);
            if (center != null && center[1] > UP_CELL_MIN_Y) {
                count++;
            }
        }
        return count;
    }

    /**
     * The CELL a tag sits under, or null if the tag is not one of this alliance's.
     *
     * <p>TU06 9.9 numbers the clusters in field order: 30-33 on the red CELL on the side opposite
     * the audience (JUDGE_SIDE), 34-37 on the red CELL on the AUDIENCE_SIDE, 38-41 on the blue
     * CELL on the AUDIENCE_SIDE and 42-45 on the blue CELL on the JUDGE_SIDE. The layout is
     * mirrored between alliances, so a tag's ID parity alone does not give the side: the two
     * outermost clusters are JUDGE_SIDE and the two inner ones are AUDIENCE_SIDE.
     */
    private static Cell cellOfTag(@NotNull final Alliance alliance, final int tagId) {
        if (!alliance.ownsTag(tagId)) {
            return null;
        }
        final int cell = Alliance.cellOfTag(tagId);
        return cell == 0 || cell == 3 ? Cell.JUDGE_SIDE : Cell.AUDIENCE_SIDE;
    }

    /**
     * The tag to aim at: the largest tag of the UP CELL when that is known, otherwise the largest
     * of this alliance's visible tags. Returns null without a usable result.
     */
    private LLResultTypes.FiducialResult selectAimTag(@NotNull final Alliance alliance) {
        final LLResult live = freshResult();
        if (live == null) {
            return null;
        }
        final Cell up = getUpCell(alliance);
        final LLResultTypes.FiducialResult ofUpCell = up == null ? null : selectAimTag(alliance, up);
        if (ofUpCell != null) {
            return ofUpCell;
        }
        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult tag : live.getFiducialResults()) {
            if (tag.getTargetArea() < TAG_MIN_AREA || !alliance.ownsTag(tag.getFiducialId())) {
                continue;
            }
            if (best == null || tag.getTargetArea() > best.getTargetArea()) {
                best = tag;
            }
        }
        return best;
    }

    private LLResultTypes.FiducialResult selectAimTag(@NotNull final Alliance alliance,
                                                      @NotNull final Cell side) {
        final LLResult live = freshResult();
        if (live == null) {
            return null;
        }
        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult tag : live.getFiducialResults()) {
            if (tag.getTargetArea() < TAG_MIN_AREA) {
                continue;
            }
            if (cellOfTag(alliance, tag.getFiducialId()) != side) {
                continue;
            }
            if (best == null || tag.getTargetArea() > best.getTargetArea()) {
                best = tag;
            }
        }
        return best;
    }

    /** Centre of a tag's 3D bounding box in normalized crop coordinates, or null. */
    private static double[] tagCenter(@NotNull final LLResultTypes.FiducialResult tag) {
        final List<List<Double>> corners = tag.getTargetCorners();
        if (corners == null || corners.isEmpty()) {
            return null;
        }
        double sumX = 0.0;
        double sumY = 0.0;
        int count = 0;
        for (List<Double> corner : corners) {
            if (corner == null || corner.size() < 2
                    || corner.get(0) == null || corner.get(1) == null) {
                continue;
            }
            sumX += corner.get(0);
            sumY += corner.get(1);
            count++;
        }
        return count == 0 ? null : new double[]{sumX / count, sumY / count};
    }

    /**
     * The tag's orientation as a rotation from tag coordinates into camera coordinates, or null
     * if the Limelight has no pose for it. Camera space keeps this independent of the turret angle
     * and of the configured mounting transform.
     */
    private static double[][] tagRotation(@NotNull final LLResultTypes.FiducialResult tag) {
        final Pose3D pose = tag.getTargetPoseCameraSpace();
        if (pose == null) {
            return null;
        }
        return rotationMatrix(pose.getOrientation());
    }

    /**
     * {@code Rz(yaw) * Ry(pitch) * Rx(roll)}, the convention the 3D-box pose uses. Its third
     * column is the tag's face normal, which is why a baseline and a current frame captured the
     * same way compare directly.
     */
    private static double[][] rotationMatrix(@NotNull final YawPitchRollAngles orientation) {
        final double yaw = orientation.getYaw(AngleUnit.RADIANS);
        final double pitch = orientation.getPitch(AngleUnit.RADIANS);
        final double roll = orientation.getRoll(AngleUnit.RADIANS);
        final double cy = Math.cos(yaw), sy = Math.sin(yaw);
        final double cp = Math.cos(pitch), sp = Math.sin(pitch);
        final double cr = Math.cos(roll), sr = Math.sin(roll);
        return new double[][]{
                {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
                {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
                {-sp, cp * sr, cp * cr}
        };
    }

    /** {@code current * baseline^T}: the rotation that carries the baseline onto the current. */
    private static double[][] relativeRotation(final double @NotNull [] @NotNull [] current,
                                               final double @NotNull [] @NotNull [] baseline) {
        final double[][] relative = new double[3][3];
        for (int row = 0; row < 3; row++) {
            for (int column = 0; column < 3; column++) {
                double sum = 0.0;
                for (int k = 0; k < 3; k++) {
                    sum += current[row][k] * baseline[column][k];
                }
                relative[row][column] = sum;
            }
        }
        return relative;
    }

    /**
     * Axis-angle of a rotation matrix as a vector in degrees, {@code k * theta}. The skew part of
     * {@code R} is {@code 2 sin(theta) [k]x}, which is exact except at 180 deg, far outside what a
     * HIVE can do between two frames.
     */
    private static double[] rotationVector(final double @NotNull [] @NotNull [] rotation) {
        final double trace = rotation[0][0] + rotation[1][1] + rotation[2][2];
        final double cosine = Math.max(-1.0, Math.min(1.0, (trace - 1.0) / 2.0));
        final double angle = Math.acos(cosine);
        if (angle < 1e-9) {
            return new double[]{0.0, 0.0, 0.0};
        }
        final double scale = Math.toDegrees(angle) / (2.0 * Math.sin(angle));
        return new double[]{
                (rotation[2][1] - rotation[1][2]) * scale,
                (rotation[0][2] - rotation[2][0]) * scale,
                (rotation[1][0] - rotation[0][1]) * scale
        };
    }

    /**
     * The world vertical in camera coordinates, {@code (0, -cos(pitch), sin(pitch))}. No yaw term
     * appears because the turret turns about that same vertical, which is what makes the
     * decomposition valid at every turret angle.
     */
    private static double[] cameraUp() {
        final double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        return new double[]{0.0, -Math.cos(pitch), Math.sin(pitch)};
    }

    /**
     * The field's red-blue axis (+x) in camera coordinates, from the camera yaw in the field frame:
     * <pre>
     * a = (sin(psi), sin(pitch) * cos(psi), cos(pitch) * cos(psi))
     * </pre>
     * Null while the camera yaw is unknown.
     */
    private double[] pivotAxis() {
        if (Double.isNaN(cameraYawFieldDeg)) {
            return null;
        }
        final double yaw = Math.toRadians(cameraYawFieldDeg);
        final double pitch = Math.toRadians(CAMERA_PITCH_DEG);
        return new double[]{
                Math.sin(yaw),
                Math.sin(pitch) * Math.cos(yaw),
                Math.cos(pitch) * Math.cos(yaw)
        };
    }

    /**
     * Names the descending CELL from the horizontal rotation: a right-handed rotation about +x
     * lifts the +y CELL, and +y is the JUDGE_SIDE, so a positive projection takes the
     * AUDIENCE_SIDE CELL down. A direction that contradicts the watched UP CELL, or a rotation
     * that is not about the pivot axis, is reported as UNKNOWN rather than guessed.
     */
    private TiltDirection directionOf(final double @NotNull [] horizontal) {
        final double[] pivot = pivotAxis();
        if (pivot == null) {
            return TiltDirection.UNKNOWN;
        }
        final double alongPivot = dot(horizontal, pivot);
        final double alignment = Math.abs(alongPivot) / Math.max(norm(horizontal), 1e-9);
        if (alignment < 0.8) {
            // The HIVE cannot swing about the field's other horizontal axis; this is a bad pose.
            return TiltDirection.UNKNOWN;
        }
        final Cell descending = alongPivot > 0.0 ? Cell.AUDIENCE_SIDE : Cell.JUDGE_SIDE;
        if (watchedCell != null && descending != watchedCell) {
            // A tip always takes the UP CELL down; anything else is a measurement fault.
            return TiltDirection.UNKNOWN;
        }
        return descending == Cell.AUDIENCE_SIDE ? TiltDirection.AUDIENCE_SIDE
                : TiltDirection.JUDGE_SIDE;
    }

    private static double dot(final double @NotNull [] a, final double @NotNull [] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    private static double norm(final double @NotNull [] vector) {
        return Math.sqrt(dot(vector, vector));
    }
}
