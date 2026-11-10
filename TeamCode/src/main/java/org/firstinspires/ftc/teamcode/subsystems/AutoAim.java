package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.utils.Alliance;
import org.jetbrains.annotations.NotNull;

/**
 * Chassis auto-aim for the fixed shooter: owns the Limelight 3A and turns the tag cluster
 * under one of our HIVE CELLS into a turn command for the drivetrain.
 *
 * <p>Every new camera frame sets a target heading; between frames the OctoQuad heading
 * closes the loop, so the robot keeps steering at camera rate or faster and the driver can
 * keep translating field-centric while the chassis stays pointed at the CELL. The driver
 * decides when to use {@link #getTurnPower()}; this class never moves anything itself.
 *
 * <p>Nothing here is calibrated on the robot yet. Before trusting it: check that a tag to
 * the right of the crosshair reads positive tx, that the robot turns toward the tag (else
 * set INVERT_TURN), then tune AIM_TX_DEG and the gains in Panels at low MAX_TURN. ALIGNED
 * is an angle check only: it knows nothing about range, flywheel speed or which CELL is
 * tilted up.
 */
@Configurable
public final class AutoAim extends SubsystemBase {
    // AprilTag pipeline index on the Limelight; results from any other pipeline are rejected.
    public static int PIPELINE = 0;
    // Tag-cluster tx in degrees (positive = right of the crosshair) at which shots land.
    public static double AIM_TX_DEG = 0.0;
    // Turn power per degree of heading error, and the cap on the assist's turn power.
    public static double TURN_KP = 0.015;
    public static double MAX_TURN = 0.25;
    // Extra turn power outside the tolerance to break static friction; 0 disables it.
    public static double TURN_KS = 0.0;
    public static double TOLERANCE_DEG = 1.5;
    // The error must stay inside the tolerance this long before ALIGNED is reported.
    public static double SETTLE_SECONDS = 0.15;
    // Frames older than this are ignored. 100 ms is Limelight's sample value, not measured here.
    public static double MAX_FRAME_AGE_MS = 100.0;
    // Without a fresh frame for this long the target is dropped and the driver steers again.
    public static double LOCK_HOLD_SECONDS = 0.4;
    // Stay on the current CELL unless another one's tags cover this many times more image area.
    public static double CELL_SWITCH_AREA_RATIO = 1.5;
    // Set true if the robot turns away from the tag instead of toward it.
    public static boolean INVERT_TURN = false;
    // Right-stick deflection at which the driver takes the turn back from the assist.
    public static double MANUAL_OVERRIDE = 0.12;

    private final Limelight3A limelight;
    private final Localizer localizer;
    private final ElapsedTime sinceFrame = new ElapsedTime();
    private final ElapsedTime sinceOutOfTolerance = new ElapsedTime();
    private Alliance alliance;
    private double lastFrameStamp = Double.NaN;
    // Target in raw OctoQuad heading degrees; NaN while there is no target.
    private double targetHeadingDeg = Double.NaN;
    private double errorDeg = Double.NaN;
    private double txDeg = Double.NaN;
    private int targetCell = -1;
    private String status;

    /** A robot configuration without a device named "limelight" only disables the assist. */
    public AutoAim(@NotNull final HardwareMap hMap, @NotNull final Localizer localizer) {
        this.localizer = localizer;
        limelight = hMap.tryGet(Limelight3A.class, "limelight");
        if (limelight == null) {
            status = "NO LIMELIGHT IN CONFIG";
            return;
        }
        limelight.setPollRateHz(100);
        // Fire-and-forget: periodic() checks each result's pipeline index before using it.
        limelight.pipelineSwitch(PIPELINE);
        limelight.start();
        status = "SELECT ALLIANCE";
    }

    /** Only this alliance's CELL tags are aimed at; null disables the assist. */
    public void setAlliance(Alliance alliance) {
        if (this.alliance != alliance) {
            this.alliance = alliance;
            dropTarget(alliance == null ? "SELECT ALLIANCE" : "NO TARGET");
        }
    }

    public Alliance getAlliance() {
        return alliance;
    }

    @Override
    public void periodic() {
        if (limelight == null) {
            return;
        }
        if (alliance == null) {
            dropTarget("SELECT ALLIANCE");
            return;
        }
        double headingDeg = Math.toDegrees(localizer.getRawHeadingRadians());
        if (!Double.isFinite(headingDeg)) {
            dropTarget("NO HEADING");
            return;
        }

        LLResult result = limelight.getLatestResult();
        // Polling faster than the camera returns the same frame again: act on it only once.
        if (isUsable(result) && result.getTimestamp() != lastFrameStamp) {
            lastFrameStamp = result.getTimestamp();
            double tx = selectTargetTx(result);
            if (Double.isFinite(tx)) {
                // tx is positive to the right; heading is positive counterclockwise.
                targetHeadingDeg = headingDeg - (tx - AIM_TX_DEG);
                txDeg = tx;
                sinceFrame.reset();
            }
        }
        if (!Double.isFinite(targetHeadingDeg) || sinceFrame.seconds() > LOCK_HOLD_SECONDS) {
            dropTarget("NO TARGET");
            return;
        }

        errorDeg = wrapDegrees(targetHeadingDeg - headingDeg);
        if (Math.abs(errorDeg) > TOLERANCE_DEG) {
            sinceOutOfTolerance.reset();
        }
        status = isAligned() ? "ALIGNED" : "TRACKING";
    }

    /**
     * Turn power for {@code Drive.drive} (positive = clockwise) that points the shooter at
     * the target, or NaN when there is no fresh target and the driver must steer.
     */
    public double getTurnPower() {
        if (!hasTarget()) {
            return Double.NaN;
        }
        if (Math.abs(errorDeg) <= TOLERANCE_DEG) {
            return 0.0;
        }
        double counterclockwise = TURN_KP * errorDeg + Math.copySign(TURN_KS, errorDeg);
        counterclockwise = Math.max(-MAX_TURN, Math.min(MAX_TURN, counterclockwise));
        return INVERT_TURN ? counterclockwise : -counterclockwise;
    }

    public boolean hasTarget() {
        return Double.isFinite(errorDeg);
    }

    /** Heading error has stayed inside TOLERANCE_DEG for SETTLE_SECONDS. Angle only. */
    public boolean isAligned() {
        return hasTarget() && Math.abs(errorDeg) <= TOLERANCE_DEG
                && sinceOutOfTolerance.seconds() >= SETTLE_SECONDS;
    }

    /** Degrees the robot still has to turn, positive = counterclockwise; NaN without a target. */
    public double getErrorDeg() {
        return errorDeg;
    }

    /** tx of the target in the last used frame, degrees; NaN without a target. */
    public double getTxDeg() {
        return txDeg;
    }

    public String getStatus() {
        return status;
    }

    public boolean isCameraConnected() {
        return limelight != null && limelight.isConnected();
    }

    /** Stops the Limelight polling thread; call when the OpMode ends. */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
        dropTarget("STOPPED");
    }

    private static boolean isUsable(LLResult result) {
        if (result == null || !result.isValid() || result.getPipelineIndex() != PIPELINE) {
            return false;
        }
        long ageMs = result.getStaleness();
        return ageMs >= 0 && ageMs <= MAX_FRAME_AGE_MS;
    }

    /**
     * Mean tx of the visible tags under one of our CELLS, or NaN if none is visible.
     * Averaging the cluster centres the aim on the CELL when all four tags are seen; a
     * partly hidden cluster shifts it by up to half the cluster width.
     */
    private double selectTargetTx(LLResult result) {
        double[] area = new double[Alliance.CELL_COUNT];
        double[] txSum = new double[Alliance.CELL_COUNT];
        int[] tags = new int[Alliance.CELL_COUNT];
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int cell = Alliance.cellOfTag(tag.getFiducialId());
            if (cell < 0 || !alliance.ownsTag(tag.getFiducialId())) {
                continue;
            }
            area[cell] += tag.getTargetArea();
            txSum[cell] += tag.getTargetXDegrees();
            tags[cell]++;
        }

        int best = -1;
        for (int cell = 0; cell < Alliance.CELL_COUNT; cell++) {
            if (tags[cell] > 0 && (best < 0 || area[cell] > area[best])) {
                best = cell;
            }
        }
        if (best < 0) {
            return Double.NaN;
        }
        // Two CELLS of similar size in view must not make the target jump between frames.
        if (targetCell >= 0 && tags[targetCell] > 0
                && area[best] < CELL_SWITCH_AREA_RATIO * area[targetCell]) {
            best = targetCell;
        }
        targetCell = best;
        return txSum[best] / tags[best];
    }

    private void dropTarget(String newStatus) {
        targetHeadingDeg = Double.NaN;
        errorDeg = Double.NaN;
        txDeg = Double.NaN;
        targetCell = -1;
        // A new target has to settle again before it counts as aligned.
        sinceOutOfTolerance.reset();
        status = newStatus;
    }

    private static double wrapDegrees(double degrees) {
        double radians = Math.toRadians(degrees);
        return Math.toDegrees(Math.atan2(Math.sin(radians), Math.cos(radians)));
    }
}
