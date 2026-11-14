package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.jetbrains.annotations.NotNull;

/**
 * Shared control mappings; reads the current gamepad state on every call.
 *
 * <p>Gamepad 1 (driver): driving, the splitter, the drive-to-shooting-position buttons and the
 * odometry reset. Gamepad 2 (operator): LB collects; RB spins up the shooter; RT runs the
 * shooter and feeds only when the gate can open. Intake (LB) and outtake (LT) work from either gamepad.
 */
public final class GamepadConfig {
    public static final double DRIVE_DEADBAND = 0.05;
    // Translation input multiplier for the left stick: 1.0 uses full power. The chassis is
    // light, so full power is hard to control. 0.7 is a starting value, not tuned.
    public static final double DRIVE_LIMIT = 1.0;
    // Turn input multiplier: 0.0 disables turning, 1.0 uses full turn power.
    public static final double TURN_LIMIT = 0.7;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    public GamepadConfig(@NotNull final Gamepad gamepad1, @NotNull final Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    // ---------- Gamepad 1: driver ----------

    /**
     * LB on either gamepad (hold): intake. If one
     * gamepad holds intake and the other outtake, intake wins.
     */
    public boolean isIntakeHeld() {
        return gamepad1.left_bumper || gamepad2.left_bumper;
    }

    /** LT on either gamepad (hold): outtake. */
    public boolean isOuttakeHeld() {
        return gamepad1.left_trigger_pressed || gamepad2.left_trigger_pressed;
    }

    public double getForward() {
        return -gamepad1.left_stick_y;
    }

    public double getStrafe() {
        return gamepad1.left_stick_x;
    }

    public double getTurn() {
        return gamepad1.right_stick_x;
    }

    /** Gamepad 1 D-pad up (press): toggle Field-Centric's emergency robot-centric mode. */
    public boolean isDriveModeToggleHeld() {
        return gamepad1.dpad_up;
    }

    /** Gamepad 1 A: toggles the splitter down / up. */
    public boolean isSplitterHeld() {
        return gamepad1.a;
    }

    /**
     * Gamepad 1 D-pad down (press): resets the odometry pose to the alliance's reset pose
     * (Field-Centric only).
     */
    public boolean isResetPoseHeld() {
        return gamepad1.dpad_down;
    }

    /** B: drive to the first shooting position (Field-Centric only). */
    public boolean isDriveToShoot1Held() {
        return gamepad1.b;
    }

    /** X: drive to the second shooting position (Field-Centric only). */
    public boolean isDriveToShoot2Held() {
        return gamepad1.x;
    }

    /** INIT only: X (blue button) selects the BLUE alliance. */
    public boolean isBlueAllianceSelected() {
        return gamepad1.x;
    }

    /** INIT only: B (red button) selects the RED alliance. */
    public boolean isRedAllianceSelected() {
        return gamepad1.b;
    }

    public void rumble(int durationMs) {
        gamepad1.rumble(durationMs);
    }

    // ---------- Gamepad 2: operator ----------

    /** Gamepad 2 RB (hold): runs the shooter only. */
    public boolean isShootHeld() {
        return gamepad2.right_bumper;
    }

    /** Gamepad 2 RT (hold): runs the shooter and feeds only when canOpenGate() is true. */
    public boolean isFireHeld() {
        return gamepad2.right_trigger_pressed;
    }
}
