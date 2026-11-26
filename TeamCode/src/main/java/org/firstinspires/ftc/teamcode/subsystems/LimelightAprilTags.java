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
 * AprilTag detection services on the fixed upward Limelight: which CELL of the HIVE faces
 * the camera, which way the HIVE tips after a shot, and where the turret has to point.
 *
 * <p>The pipeline is configured at limelight.local:5801 as an AprilTag detector that reports
 * each tag as a 3D bounding box, cropped to normalized Y in {@code [-0.25, 1]}. Cropping
 * to the top of the frame is what makes a CELL identifiable: the bottom row of the UP CELL
 * sits just inside the upper half of the cropped input, so about half of that CELL's
 * {@link Alliance#TAGS_PER_CELL} tags are detected there, and the DOWN CELL's tags are not
 * detected at all. A single upper-half tag is therefore evidence of UP, and the tags moving
 * up out of the crop is evidence that the HIVE is falling.
 *
 * <p>Three independent services sit on one sensor; they never own hardware other than the
 * camera and never decide when to shoot. The OpMode, or a turret subsystem that is added
 * separately, reads them:
 * <ul>
 * <li><b>CELL state.</b> {@link #getCellPosition} and {@link #getUpCell} answer which CELL is
 *     shootable now, for aiming and for refusing to feed a DOWN CELL.</li>
 * <li><b>Tilt prediction.</b> A tipped HIVE is worth 20 points but takes about 3 s to fall, so
 *     {@link #startTiltObservation} right after the shot and {@link #updateTiltPrediction} in
 *     the loop reports TILTING as soon as the fall is visible, instead of waiting at the end of
 *     AUTO. Every tag turns with the HIVE, so the turn of one watched tag normal by
 *     {@link #TILT_CONFIRM_DEG}, or that tag climbing {@link #TILT_MIN_DRIFT} out of the crop,
 *     is enough. {@link #getTiltDirection} adds which side the HIVE falls toward, so a routine
 *     can tell a partner's tip from its own and stop shooting into a CELL that has left the
 *     shooting pose (POLLEN loss).</li>
 * <li><b>Turret aim assist.</b> {@link #getAimYawErrorDeg} is the signed turret correction that
 *     centres an AprilTag in AUTO or TELEOP; {@link #isAimedAt} closes the loop. The turret
 *     subsystem owns its motor, its zero, and its gear sign, so this class reports only the
 *     measured error.</li>
 * </ul>
 *
 * <p>Every consumer passes one gate: a result is used only when the camera is connected, the
 * result is valid, the frame is younger than {@link #MAX_RESULT_AGE_MS}, and the camera
 * reports the AprilTag pipeline. {@link Limelight3A#pipelineSwitch} is fire-and-forget, so the
 * pipeline index is read back from the result rather than assumed.
 * Camera pose and lens calibration are configured on the Limelight itself; {@link
 * #setRobotHeadingDeg} supplies the current heading for field-space output, but nothing here
 * consumes MegaTag 2 poses.
 *
 * <p>Tag geometry comes from the 3D bounding box, not from tx/ty alone: an image position moves
 * whenever the camera does, an orientation only changes when the tag does. The direction of a
 * tip is the horizontal drift of the watched tags, so {@link #TILT_TOWARD_AUDIENCE_DRIFT_SIGN}
 * is a field-calibration constant for the camera mounting, not a robot-independent fact.
 */
@Configurable
public final class LimelightAprilTags extends SubsystemBase {
    // Pipeline ID must match the configuration at limelight.local:5801.
    public static String LIMELIGHT_NAME = "limelight";
    public static int PIPELINE_APRILTAG = 0;
    public static int POLL_RATE_HZ = 100;

    // A result older than this cannot be used for a control decision: a stale frame would
    // report the previous pose, which is exactly what a tilt or an aim correction must not do.
    public static long MAX_RESULT_AGE_MS = 250;

    // With the pipeline cropped to normalized Y in [-0.25, 1], a detected tag at or above this
    // line counts as upper-half evidence of the UP CELL. Measured on the robot: the bottom row
    // of the UP CELL lands just above it, and the DOWN CELL's tags never appear at all.
    public static double UP_CELL_MIN_Y = 0.0;

    // A tag at least this much of the image is worth trusting; smaller targets are noise.
    public static double TAG_MIN_AREA = 0.1;
    // One CELL carries this many tags, so more visible tags cannot all be on one UP CELL.
    public static int MAX_TAGS_AT_SHOOTING_POS = Alliance.TAGS_PER_CELL;

    // Turn of a tag normal that proves the HIVE is tipping.
    public static double TILT_CONFIRM_DEG = 10.0;
    // Normalized-image drift of a watched tag that proves the same thing when the normal is
    // still settling: the bottom tags of the UP CELL climb out of the crop as it falls.
    public static double TILT_MIN_DRIFT = 0.08;
    // Which sign of horizontal tag drift means the HIVE falls toward the AUDIENCE_SIDE.
    // Set to -1 on a mounting where the tags drift the other way; this is field-calibrated.
    public static double TILT_TOWARD_AUDIENCE_DRIFT_SIGN = 1.0;
    // How long to watch for a tip. A real tip reaches the threshold in well under a second,
    // so the window mostly bounds the wait when the HIVE is not tipping.
    public static long TILT_WINDOW_MS = 2500;
    // A tag can drop out for a frame; once the HIVE falls, its tags stay gone.
    public static long TAG_LOST_MS = 100;

    // Aim assist: while the error is inside this band the turret counts as on target.
    public static double AIM_TOLERANCE_DEG = 0.75;

    /** One of the two CELLS of a HIVE, named for the side of the field it faces. */
    public enum Cell { AUDIENCE_SIDE, NON_AUDIENCE_SIDE }

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

    /** Which side the UP CELL is falling toward. */
    public enum TiltDirection {
        /** A tilt was confirmed, but the drift is too small or too noisy to name a side. */
        UNKNOWN,
        /** No tilt was observed through the window. */
        NONE,
        /** The UP CELL is falling toward the AUDIENCE_SIDE of the field. */
        AUDIENCE_SIDE,
        /** The UP CELL is falling toward the NON_AUDIENCE_SIDE of the field. */
        NON_AUDIENCE_SIDE
    }

    private final Limelight3A limelight;
    private LLResult result;

    // What a tilt observation is watching, captured when it starts: the tag normal and the
    // normalized image centre of each upper-half tag of the UP CELL.
    private final Map<Integer, double[]> baselineNormals = new HashMap<>();
    private final Map<Integer, double[]> baselineCenters = new HashMap<>();
    private final ElapsedTime sinceObservationStart = new ElapsedTime();
    private final ElapsedTime sinceTagsSeen = new ElapsedTime();
    private TiltPrediction tiltPrediction = TiltPrediction.UNKNOWN;
    private TiltDirection tiltDirection = TiltDirection.UNKNOWN;
    private double maxAngleChangeDeg;
    private double meanDriftX;
    private double maxDriftY;

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

    // ---------- Result validity and freshness ----------

    /**
     * The latest result if it can be used right now, else null: connected, valid, inside the
     * age policy, and reporting the AprilTag pipeline.
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
    public CellPosition getCellPosition(@NotNull final Alliance alliance, @NotNull final Cell side) {
        final LLResult live = freshResult();
        if (live == null) {
            return CellPosition.UNKNOWN;
        }
        final int audience = countUpCellTags(live, alliance, Cell.AUDIENCE_SIDE);
        final int nonAudience = countUpCellTags(live, alliance, Cell.NON_AUDIENCE_SIDE);
        final int total = audience + nonAudience;
        if (total > MAX_TAGS_AT_SHOOTING_POS) {
            return CellPosition.NOT_AT_SHOOTING_POSITION;
        }
        if (total == 0) {
            // No CELL tag at all means "not looking at a shooting pose", not "our CELL is
            // DOWN": the DOWN CELL's tags are invisible too, so DOWN cannot be claimed here.
            return CellPosition.UNKNOWN;
        }
        if (audience > 0 && nonAudience > 0) {
            return CellPosition.UNKNOWN;
        }
        final Cell up = audience > 0 ? Cell.AUDIENCE_SIDE : Cell.NON_AUDIENCE_SIDE;
        return side == up ? CellPosition.UP : CellPosition.DOWN;
    }

    /** The CELL that is UP, or null if neither is or the tags cannot tell. */
    public Cell getUpCell(@NotNull final Alliance alliance) {
        final boolean audienceUp = getCellPosition(alliance, Cell.AUDIENCE_SIDE) == CellPosition.UP;
        final boolean nonAudienceUp =
                getCellPosition(alliance, Cell.NON_AUDIENCE_SIDE) == CellPosition.UP;
        if (audienceUp == nonAudienceUp) {
            return null;
        }
        return audienceUp ? Cell.AUDIENCE_SIDE : Cell.NON_AUDIENCE_SIDE;
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
    public void startTiltObservation(@NotNull final Alliance alliance, @NotNull final Cell upSide) {
        baselineNormals.clear();
        baselineCenters.clear();
        maxAngleChangeDeg = 0.0;
        meanDriftX = 0.0;
        maxDriftY = 0.0;
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
            baselineNormals.put(tag.getFiducialId(), tagNormal(tag));
            baselineCenters.put(tag.getFiducialId(), center);
        }
        if (baselineNormals.isEmpty()) {
            return;
        }
        sinceObservationStart.reset();
        sinceTagsSeen.reset();
        tiltPrediction = TiltPrediction.OBSERVING;
    }

    /**
     * Polls the tilt prediction; call once per loop while it is not terminal. Returns
     * OBSERVING until the watched tags can confirm or rule out a tip, and UNKNOWN if the
     * frame becomes unusable before anything could be concluded.
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

        double maxAngleDeg = 0.0;
        double sumDriftX = 0.0;
        double largestDriftY = 0.0;
        int visible = 0;
        for (LLResultTypes.FiducialResult tag : live.getFiducialResults()) {
            final int id = tag.getFiducialId();
            final double[] baselineNormal = baselineNormals.get(id);
            final double[] baselineCenter = baselineCenters.get(id);
            if (baselineNormal == null || baselineCenter == null
                    || tag.getTargetArea() < TAG_MIN_AREA) {
                continue;
            }
            final double[] center = tagCenter(tag);
            if (center == null) {
                continue;
            }
            maxAngleDeg = Math.max(maxAngleDeg, angleBetween(baselineNormal, tagNormal(tag)));
            sumDriftX += center[0] - baselineCenter[0];
            largestDriftY = Math.max(largestDriftY, Math.abs(center[1] - baselineCenter[1]));
            visible++;
        }
        maxAngleChangeDeg = maxAngleDeg;
        meanDriftX = visible > 0 ? sumDriftX / visible : 0.0;
        maxDriftY = largestDriftY;

        if (visible > 0) {
            sinceTagsSeen.reset();
        }

        if (maxAngleDeg >= TILT_CONFIRM_DEG || largestDriftY >= TILT_MIN_DRIFT) {
            tiltDirection = directionOf(meanDriftX);
            return tiltPrediction = TiltPrediction.TILTING;
        }
        if (visible == 0 && sinceTagsSeen.milliseconds() >= TAG_LOST_MS) {
            // At a held pose nothing but the HIVE falling can keep the UP CELL's tags out of
            // view, so their loss is the strongest tilt signal there is.
            tiltDirection = directionOf(meanDriftX);
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
     * True when there is a usable result with one of this alliance's tags to aim at, so an
     * AUTO or TELEOP aim assist can act on {@link #getAimYawErrorDeg}.
     */
    public boolean hasAimTarget(@NotNull final Alliance alliance) {
        return selectAimTag(alliance) != null;
    }

    /**
     * Signed yaw error, in degrees, that centres the aim target: positive means the turret has
     * to rotate counter-clockwise (robot's left), negative clockwise. The turret subsystem adds
     * its own zero and gear sign; this class never commands a motor. NaN when there is no
     * usable result or no tag to aim at, so a caller must check {@link #hasAimTarget} first.
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

    /** Which side the UP CELL is falling toward; NONE when the window ruled a tip out. */
    public TiltDirection getTiltDirection() {
        return tiltDirection;
    }

    /** Largest turn of a watched tag normal since the observation started, in degrees. */
    public double getMaxAngleChangeDeg() {
        return maxAngleChangeDeg;
    }

    /** Mean horizontal drift of the watched tags, in normalized crop units. */
    public double getMeanDriftX() {
        return meanDriftX;
    }

    /** Largest vertical drift of the watched tags, in normalized crop units. */
    public double getMaxDriftY() {
        return maxDriftY;
    }

    /** Tags being watched, or 0 when the last observation could not start. */
    public int getBaselineTagCount() {
        return baselineNormals.size();
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

    /** The CELL a tag sits under, or null if the tag is not one of this alliance's. */
    private static Cell cellOfTag(@NotNull final Alliance alliance, final int tagId) {
        if (!alliance.ownsTag(tagId)) {
            return null;
        }
        // CELL tags are numbered audience side first within each alliance.
        return Alliance.cellOfTag(tagId) % 2 == 0 ? Cell.AUDIENCE_SIDE : Cell.NON_AUDIENCE_SIDE;
    }

    /**
     * The tag to aim at: the largest tag of the UP CELL when that is known, otherwise the
     * largest of this alliance's visible tags. Returns null without a usable result.
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
     * The tag's face normal in robot space.
     *
     * <p>Limelight's target space puts +z out of the tag's face, and the tag pose is reported as
     * yaw, pitch and roll, so the normal is the tag's +z axis carried into robot space: the
     * third column of {@code Rz(yaw) * Ry(pitch) * Rx(roll)}.
     * <pre>
     * nx = cy*sp*cr + sy*sr
     * ny = sy*sp*cr - cy*sr
     * nz = cp*cr
     * </pre>
     * Baseline and current normals are built the same way, so a tag that has not moved always
     * produces the same normal and only a real movement can change the angle.
     */
    private static double[] tagNormal(@NotNull final LLResultTypes.FiducialResult tag) {
        final YawPitchRollAngles orientation = tag.getTargetPoseRobotSpace().getOrientation();
        final double yaw = orientation.getYaw(AngleUnit.RADIANS);
        final double pitch = orientation.getPitch(AngleUnit.RADIANS);
        final double roll = orientation.getRoll(AngleUnit.RADIANS);
        final double cy = Math.cos(yaw), sy = Math.sin(yaw);
        final double cp = Math.cos(pitch), sp = Math.sin(pitch);
        final double cr = Math.cos(roll), sr = Math.sin(roll);
        return new double[]{
                cy * sp * cr + sy * sr,
                sy * sp * cr - cy * sr,
                cp * cr
        };
    }

    /** Angle between two unit vectors, in degrees. */
    private static double angleBetween(final double @NotNull [] a, final double @NotNull [] b) {
        final double dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        return Math.toDegrees(Math.acos(Math.max(-1.0, Math.min(1.0, dot))));
    }

    /** Maps horizontal tag drift to the side the HIVE is falling toward. */
    private static TiltDirection directionOf(final double meanDrift) {
        if (Math.abs(meanDrift) < TILT_MIN_DRIFT) {
            return TiltDirection.UNKNOWN;
        }
        final boolean towardAudience = meanDrift * TILT_TOWARD_AUDIENCE_DRIFT_SIGN > 0.0;
        return towardAudience ? TiltDirection.AUDIENCE_SIDE : TiltDirection.NON_AUDIENCE_SIDE;
    }
}
