package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.NotNull;

/**
 * 云台：一个 Axon Mini MK2 舵机通过齿轮带动云台转动。
 *
 * <h3>两种“角度”，不要混淆</h3>
 * <ul>
 * <li><b>舵机位置</b>（0.0 ~ 1.0）：发给舵机的指令。0.5 是舵机行程的正中间。</li>
 * <li><b>云台角度</b>（本类所有方法用的角度）：相对机器人本身，逆时针为正：
 *     0° = 车头，90° = 车左侧，-90°（= 270°）= 车右侧，180° = 车尾。</li>
 * </ul>
 *
 * <h3>当前的对应关系</h3>
 * 物理校准方法：调用 {@link #lockTurret()} 让舵机停在 0.5，然后把云台齿轮对到车头再啮合。
 * 所以舵机 0.5 = 云台 0°（车头），云台只能在车头左右各 90° 内转动：
 * <pre>
 *   舵机位置    云台角度    实际方向
 *   0.05        -90°        车右侧
 *   0.50          0°        车头（待机 / 锁定）
 *   0.95        +90°        车左侧
 * </pre>
 * 车尾方向（90°~270° 之间那半圈）转不到，目标在那里时云台会停在左或右 90° 的极限。
 *
 * <h3>换算公式</h3>
 * 舵机位置 = CENTER_POSITION + 云台偏移角 / TURRET_DEGREES_PER_SERVO_RANGE，
 * 其中 TURRET_DEGREES_PER_SERVO_RANGE = 舵机全行程 / 齿轮比 = 360° / 1.8 = 200°，
 * 即舵机位置从 0 走到 1，云台转 200°。所以 ±90° 对应舵机 0.5 ± 0.45。
 */
@Configurable
public final class Turret extends SubsystemBase {
    // 云台居中（舵机在 CENTER_POSITION）时，云台指向的方向，用“云台角度”表示。
    // 0° = 车头。改了物理对中方向才需要改这里。
    private static final double CENTER_ANGLE_DEGREES = 0.0;
    // 云台从居中位置向左、向右各最多转多少度。
    private static final double MAX_OFFSET_DEGREES = 90.0;
    // 齿轮比：舵机转 1.8° 云台转 1°（云台齿轮齿数 / 舵机齿轮齿数）。齿轮换了要改这里。
    private static final double SERVO_TO_TURRET_RATIO = 1.8;
    // 舵机在 PWM 500~2500 µs 之间转过的角度（Servo 模式，配合 Axon 编程器的设置）。
    // 注意：goBILDA 规格写的是 350°（0.175°/µs），这里按 360° 算，两端约差 2.5°，待实测决定。
    private static final double SERVO_TRAVEL_DEGREES = 360.0;
    // 舵机位置从 0 到 1，云台转过的角度 = 舵机全行程 / 齿轮比（当前 200°）。
    private static final double TURRET_DEGREES_PER_SERVO_RANGE =
            SERVO_TRAVEL_DEGREES / SERVO_TO_TURRET_RATIO;
    // 舵机脉宽范围：位置 0.0 = 500 µs，0.5 = 1500 µs，1.0 = 2500 µs（Axon 支持的最大范围）。
    private static final PwmControl.PwmRange SERVO_PWM_RANGE =
            new PwmControl.PwmRange(500, 2500);

    // ---------- 以下参数可以在 Panels 里实时调（重启机器人控制器 App 后恢复为这里的值） ----------

    // 云台正对车头时的舵机位置。已用 lockTurret() + 物理对中校准为 0.5。
    // 只能在 0.45~0.55 之间微调：超出后有一侧转不到 90°（代码会截断）。
    public static double CENTER_POSITION = 0.5;
    // 舵机方向。false：舵机位置变大时云台逆时针转（往车左侧）。
    // 实测给一个左边的目标，如果云台往右转，就改成 true。只在构造时读取，改完要重新 INIT。
    public static boolean REVERSED = false;
    // 防甩动区：目标在车尾附近这么多度以内时（左右两边都够不到），云台保持在当前所在的那一侧极限，
    // 不会因为航向的微小抖动在 +90° 和 -90° 之间来回甩 180°。0 = 关闭；最大 90，不影响能瞄到的角度。
    public static double REAR_HYSTERESIS_DEG = 30.0;

    private final ServoImplEx turret;
    // 最近一次命令的云台角度（不是测量值：舵机没有位置反馈）。第一次命令之前是 NaN。
    private double commandedAngleDegrees = Double.NaN;

    /** 构造时立刻让云台回中（车头）。 */
    public Turret(@NotNull final HardwareMap hMap) {
        this(hMap, true);
    }

    /**
     * @param centerOnInit true：构造时立刻回中。TeleOp 和 AUTO 都传 false，
     *                     这样 INIT 阶段舵机不会动（规则要求 INIT 后机器人静止），START 后才开始转。
     */
    public Turret(@NotNull final HardwareMap hMap, boolean centerOnInit) {
        turret = hMap.get(ServoImplEx.class, "turret");
        turret.setPwmRange(SERVO_PWM_RANGE);
        turret.setDirection(REVERSED ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        if (centerOnInit) {
            center();
        }
    }

    /** 回中：云台转到车头（舵机 CENTER_POSITION）。TeleOp 没按 RB/RT 时每个循环都会调用。 */
    public void center() {
        setAngleDegrees(CENTER_ANGLE_DEGREES);
    }

    /**
     * 自瞄：让云台指向场地上的目标点。
     *
     * <p>坐标系是队伍的 Pedro 场地坐标（英寸、弧度、逆时针为正）。方向是从机器人中心
     * （robotPose 的位置）算的，不是从云台转轴算的，所以转轴离车中心越远、目标越近，误差越大。
     *
     * @param robotPose 机器人当前位姿（里程计给的）
     * @param target    目标点（例如 HIVE 上的 cell），只用它的 x、y
     */
    public void aimAt(Pose robotPose, Pose target) {
        if (robotPose == null || target == null) {
            return;
        }
        double dx = target.x() - robotPose.x();
        double dy = target.y() - robotPose.y();
        // 位姿无效，或者机器人正好在目标点上（方向无定义）：保持上一次的指令不动。
        if (!Double.isFinite(dx) || !Double.isFinite(dy) || (dx == 0.0 && dy == 0.0)) {
            return;
        }
        // 目标在场地上的方向 - 车头在场地上的方向 = 目标相对车头的角度，也就是“云台角度”。
        double robotRelativeBearing = Math.atan2(dy, dx) - robotPose.heading();
        setAngleDegrees(Math.toDegrees(robotRelativeBearing));
    }

    /**
     * 让云台转到指定的“云台角度”（相对机器人，0° = 车头，逆时针为正）。
     * 任意角度都可以传，会先换算到 -180°~180°，再限制在 -90°~+90° 以内。
     */
    public void setAngleDegrees(double angleDegrees) {
        double centerPosition = CENTER_POSITION;
        // 角度不是有限数，或者 Panels 里把 CENTER_POSITION 调到了 0~1 以外：忽略这次指令。
        if (!Double.isFinite(angleDegrees) || !isValidCenter(centerPosition)) {
            return;
        }

        // 第 1 步：目标相对“居中方向”（车头）偏了多少度，换算到 -180°~180°。
        // 例：270° → -90°（车右侧），190° → -170°（车尾偏右）。
        double centerOffsetDegrees = AngleUnit.normalizeDegrees(angleDegrees % 360.0 - CENTER_ANGLE_DEGREES);
        // 第 2 步：限制在 ±90° 以内。超出的目标（在车尾那半圈）停在最近的极限。
        double limitedOffsetDegrees = Range.clip(centerOffsetDegrees, -MAX_OFFSET_DEGREES, MAX_OFFSET_DEGREES);
        // 第 3 步：防甩动。目标在正后方附近时，左右两边都够不到，偏移角的正负号会随航向抖动来回跳，
        // 云台就会在 +90° 和 -90° 之间甩 180°。这时保持云台当前所在的那一侧，不换边。
        double band = Range.clip(REAR_HYSTERESIS_DEG, 0.0, 180.0 - MAX_OFFSET_DEGREES);
        double currentOffsetDegrees = commandedAngleDegrees - CENTER_ANGLE_DEGREES;
        if (Math.abs(centerOffsetDegrees) > 180.0 - band && currentOffsetDegrees != 0.0
                && Double.isFinite(currentOffsetDegrees)) {
            limitedOffsetDegrees = Math.copySign(MAX_OFFSET_DEGREES, currentOffsetDegrees);
        }
        // 第 4 步：云台角度 → 舵机位置。偏移角 / 200° 就是舵机位置要偏离中位多少；
        // 再保证最终位置不超出 0~1（CENTER_POSITION 偏离 0.5 时，有一侧会被截断）。
        double offset = Range.clip(limitedOffsetDegrees / TURRET_DEGREES_PER_SERVO_RANGE,
                -centerPosition, 1.0 - centerPosition);
        turret.setPosition(centerPosition + offset);
        // 记下实际命令的云台角度（已经过限制和截断），给遥测和防甩动逻辑用。
        commandedAngleDegrees = CENTER_ANGLE_DEGREES + offset * TURRET_DEGREES_PER_SERVO_RANGE;
    }

    /**
     * 最近一次命令的云台角度（0° = 车头，-90°~+90°），第一次命令之前是 NaN。
     * 这是命令值不是测量值，遥测 "Turret angle command (deg)" 显示的就是它。
     */
    public double getCommandedAngleDegrees() {
        return commandedAngleDegrees;
    }

    /** CENTER_POSITION 必须是合法的舵机位置（0~1）。 */
    private static boolean isValidCenter(double position) {
        return position >= 0.0 && position <= 1.0;
    }

    /** 锁定云台：转到车头（舵机 CENTER_POSITION），TeleOp 按住 D-pad 左时调用。也用于物理对中校准。 */
    public void lockTurret(){
        setAngleDegrees(CENTER_ANGLE_DEGREES);
    }
}
