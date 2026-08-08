// ======================= States_Selly_Telly_Blue.java =======================
package org.firstinspires.ftc.teamcode.pedroPathing.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Testing.Subsystem.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Testing.Subsystem.TeleOp_Helper;

@Config
@TeleOp(name = "", group = "AStates")
public class Blue_Tele_Belly extends OpMode {

    // --- Full-mag reverse -> stop after 3s (non-blocking) ---
    private final ElapsedTime fullReverseTimer = new ElapsedTime();
    private boolean fullReverseActive = false;
    public static int FULL_REVERSE_MS = 3000;

    // ---------------- FOLLOWER ----------------
    public static Follower follower;
    static PoseHistory poseHistory;

    // Set these on dashboard like you do for red
    public static double START_X = 137.5;
    public static double START_Y = 8.64;

    // IMPORTANT: radians. -90deg = -PI/2
    public static double START_HEADING_RAD = -Math.PI ;

    // ✅ BLUE GOAL (you said you need to track this):
    // Start: (137.5, 8.64)  Goal: (7.5, 138)
    // IMPORTANT: keep X/Y in this order (no swap)
    static final double GOAL_X = 5;
    static final double GOAL_Y = 135.0;

    // ---------------- HARDWARE ----------------
    private Servo turret1, turret2;
    public static double ser = 0.5;

    // Driver tuning
    public static double turretOffsetDeg = -2.0;
    public static double rpmOffset = -50.0;   // dashboard trim

    // ✅ Limey helper
    private TeleOp_Helper limey;

    // ✅ Intake subsystem
    private IntakeSubsystem intake;

    // Flickers
    private Servo flick1, flick2, flick3;

    // ---------------- FLICK FSM ----------------
    private enum Phase { IDLE, FIRE, GAP }
    private Phase phase = Phase.IDLE;

    private enum FireMode { NORMAL, REMOVE }
    private FireMode fireMode = FireMode.NORMAL;

    private int[] fireOrder = new int[]{1, 2, 3};
    private int fireStep = 0;
    private final ElapsedTime phaseTimer = new ElapsedTime();
    public static int FIRE_MS = 125;
    public static int GAP_MS  = 100;

    private int activeFlicker = -1;

    // ---------------- BUTTON EDGE DETECT ----------------
    private boolean lastY = false;
    private boolean lastStart = false;

    private boolean lastG1Left = false, lastG1Right = false, lastG1Up = false, lastG1Down = false;
    private boolean lastG2Left = false, lastG2Right = false, lastG2Up = false, lastG2Down = false;

    private boolean lastFull = false;

    private double slowmode = 1;

    // ===== FLICK CONSTANTS =====
    private final double flick1Rest = 0.0, flick1Fire = 0.55;
    private double flick2Rest = 0.98, flick2Fire = 0.43;
    private double flick3Rest = 0.96, flick3Fire = 0.41;

    private final double flick1Fire2 = 0.35;
    private final double flick2Fire2 = 0.63;
    private final double flick3Fire2 = 0.61;

    // ---------------- INIT ----------------
    @Override
    public void init() {

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        flick1 = hardwareMap.get(Servo.class, "flick1");
        flick2 = hardwareMap.get(Servo.class, "flick2");
        flick3 = hardwareMap.get(Servo.class, "flick3");

        follower = Constants.createFollower(hardwareMap);

        // Pose transferred from Auto (or set via dashboard)
        follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING_RAD));
        poseHistory = follower.getPoseHistory();

        // DEBUG: confirm start + goal
        telemetry.addData("Alliance", "BLUE");
        telemetry.addData("Start (X,Y,Hdeg)", "(%.2f, %.2f, %.1f)",
                START_X, START_Y, Math.toDegrees(START_HEADING_RAD));
        telemetry.addData("Goal (X,Y)", "(%.1f, %.1f)", GOAL_X, GOAL_Y);

        // WARNING if heading smells like degrees accidentally stored as radians
        double hDeg = Math.toDegrees(START_HEADING_RAD);
        if (Math.abs(hDeg) > 360.0) {
            telemetry.addLine("WARNING: START_HEADING_RAD looks like degrees, not radians!");
            telemetry.addLine("Example: -90deg should be -PI/2 rad (~ -1.571)");
        }
        telemetry.update();

        limey = new TeleOp_Helper(hardwareMap);
        limey.start();

        intake = new IntakeSubsystem(hardwareMap);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    // ---------------- LOOP ----------------
    @Override
    public void loop() {

        intake.update();
        handleFullMagazineFeedback();
        handleDpadTuning();

        // keep limey offset synced
        limey.setRpmOffset(rpmOffset);

        // ---------------- DRIVE ----------------
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y * slowmode,
                -gamepad1.left_stick_x * slowmode,
                -gamepad1.right_stick_x * slowmode,
                true
        );
        follower.update();

        // ---------------- TURRET AIM (POSE -> SERVO) ----------------
        Pose pose = follower.getPose();
        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();
        double goalAngle = Math.atan2(dy, dx);

        // Use radians all the way, then convert once
        double turretAngleRad = goalAngle - pose.getHeading();
        turretAngleRad = wrapRadians(turretAngleRad);

        double turretAngleDeg = Math.toDegrees(turretAngleRad);

        // If you want offset enabled, uncomment:
        // turretAngleDeg += turretOffsetDeg;

        ser = 0.5 - (turretAngleDeg / 225.0);
        ser = Range.clip(ser, 0.10, 0.95);

        turret1.setPosition(ser);
        turret2.setPosition(ser);

        // ---------------- SHOOTER + LIMELIGHT AUTO ----------------
        boolean xAny = gamepad1.x || gamepad2.x;
        boolean aAny = gamepad1.a || gamepad2.a;
        limey.updateShooterToggle(xAny, aAny);

        boolean llUpdated = limey.updateAuto(true);

        // ---------------- INTAKE ----------------
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            intake.startIntake();
        } else if (gamepad1.right_trigger > 0.4 || gamepad2.left_bumper) {
            intake.stopIntake();
        } else if (gamepad1.left_bumper) {
            intake.reverseIntake();
        }

        // ---------------- FLICKER FSM ----------------
        boolean y = gamepad1.y;

        if (y && !lastY && phase == Phase.IDLE) {
            double desired = limey.getLastTargetVelocity();
            double actual  = limey.getShooterAvgVelocity();

            boolean rpmReady = Math.abs(actual - desired) <= 100.0;
            boolean canFire  = limey.isShooterEnabled() && rpmReady && desired > 0.0;

            if (canFire) {
                fireMode = FireMode.NORMAL;

                IntakeSubsystem.BallColor[] desiredPattern = new IntakeSubsystem.BallColor[]{
                        IntakeSubsystem.BallColor.PURPLE,
                        IntakeSubsystem.BallColor.GREEN,
                        IntakeSubsystem.BallColor.GREEN
                };

                fireOrder = intake.buildBestFireOrder(desiredPattern);

                fireStep = 0;
                activeFlicker = fireOrder[fireStep];
                phase = Phase.FIRE;
                phaseTimer.reset();
            }
        }
        lastY = y;

        boolean startBtn = gamepad2.start;
        if (startBtn && !lastStart && phase == Phase.IDLE) {
            fireMode = FireMode.REMOVE;

            fireOrder = new int[]{1, 2, 3};
            fireStep = 0;
            activeFlicker = fireOrder[fireStep];

            // phase = Phase.FIRE;
            // phaseTimer.reset();

            intake.reverseIntake();
        }
        lastStart = startBtn;

        switch (phase) {
            case FIRE:
                fire(activeFlicker);
                if (phaseTimer.milliseconds() >= FIRE_MS) {
                    phase = Phase.GAP;
                    phaseTimer.reset();
                }
                break;

            case GAP:
                rest(activeFlicker);
                if (phaseTimer.milliseconds() >= GAP_MS) {
                    fireStep++;
                    if (fireStep >= fireOrder.length) {
                        phase = Phase.IDLE;
                        activeFlicker = -1;
                        fireMode = FireMode.NORMAL;
                    } else {
                        activeFlicker = fireOrder[fireStep];
                        phase = Phase.FIRE;
                    }
                    phaseTimer.reset();
                }
                break;

            case IDLE:
            default:
                flick1.setPosition(flick1Rest);
                flick2.setPosition(flick2Rest);
                flick3.setPosition(flick3Rest);
                activeFlicker = -1;
                break;
        }

        // ---------------- TELEMETRY ----------------
        if (llUpdated) {
            telemetry.addData("Goal (X,Y)", "(%.1f, %.1f)", GOAL_X, GOAL_Y);
            telemetry.addData("Pose (X,Y,Hdeg)", "(%.1f, %.1f, %.1f)",
                    pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
            telemetry.addData("Target Vel (t/s)", limey.getLastTargetVelocity());
            telemetry.addData("Shooter v1 (t/s)", limey.getOuttake1Velocity());
            telemetry.addData("Shooter v2 (t/s)", limey.getOuttake2Velocity());
            telemetry.addData("Shooter vAvg (t/s)", limey.getShooterAvgVelocity());
            telemetry.update();
        }
    }

    @Override
    public void stop() {
        if (limey != null) limey.stop(true);
    }

    // ---------------- HELPERS ----------------

    private void handleFullMagazineFeedback() {
        boolean full = intake.areAllSlotsFull();

        if (full && !lastFull) {
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);

            // Start the "reverse then stop after 3s" sequence
            intake.reverseIntake();
            fullReverseActive = true;
            fullReverseTimer.reset();
        }

        if (fullReverseActive) {
            if (fullReverseTimer.milliseconds() >= FULL_REVERSE_MS) {
                intake.stopIntake();
                fullReverseActive = false;
            }
        }

        if (!full) {
            fullReverseActive = false;
        }

        lastFull = full;
    }

    private void handleDpadTuning() {
        boolean g1L = gamepad1.dpad_left;
        boolean g1R = gamepad1.dpad_right;
        boolean g1U = gamepad1.dpad_up;
        boolean g1D = gamepad1.dpad_down;

        if (g1R && !lastG1Right) turretOffsetDeg -= 1.0;
        if (g1L && !lastG1Left)  turretOffsetDeg += 1.0;
        if (g1U && !lastG1Up)    rpmOffset += 25.0;
        if (g1D && !lastG1Down)  rpmOffset -= 25.0;

        lastG1Left = g1L;
        lastG1Right = g1R;
        lastG1Up = g1U;
        lastG1Down = g1D;

        boolean g2L = gamepad2.dpad_left;
        boolean g2R = gamepad2.dpad_right;
        boolean g2U = gamepad2.dpad_up;
        boolean g2D = gamepad2.dpad_down;

        if (g2R && !lastG2Right) turretOffsetDeg -= 1.0;
        if (g2L && !lastG2Left)  turretOffsetDeg += 1.0;
        if (g2U && !lastG2Up)    rpmOffset += 25.0;
        if (g2D && !lastG2Down)  rpmOffset -= 25.0;

        lastG2Left = g2L;
        lastG2Right = g2R;
        lastG2Up = g2U;
        lastG2Down = g2D;

        turretOffsetDeg = Range.clip(turretOffsetDeg, -15.0, 15.0);
        rpmOffset = Range.clip(rpmOffset, -400.0, 400.0);
    }

    private void fire(int f) {
        if (f < 1 || f > 3) return;

        if (fireMode == FireMode.NORMAL) {
            if (f == 1) flick1.setPosition(flick1Fire);
            else if (f == 2) flick2.setPosition(flick2Fire);
            else flick3.setPosition(flick3Fire);
        } else {
            if (f == 1) flick1.setPosition(flick1Fire2);
            else if (f == 2) flick2.setPosition(flick2Fire2);
            else flick3.setPosition(flick3Fire2);
        }
    }

    private void rest(int f) {
        if (f < 1 || f > 3) return;

        if (f == 1) flick1.setPosition(flick1Rest);
        else if (f == 2) flick2.setPosition(flick2Rest);
        else flick3.setPosition(flick3Rest);
    }

    private double wrapRadians(double rad) {
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }
}