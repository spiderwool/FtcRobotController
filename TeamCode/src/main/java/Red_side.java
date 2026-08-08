// ======================= RedAfterDiddy_Jacob.java =======================
package org.firstinspires.ftc.teamcode.pedroPathing.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import org.firstinspires.ftc.teamcode.pedroPathing.Testing.Subsystem.TeleOp_Helper;
@Disabled
@Config
@TeleOp(name = "", group = "Drive")
public class Red_tele_Belly extends OpMode {

    // ---------------- FOLLOWER ----------------
    public static Follower follower;
    static PoseHistory poseHistory;

    private enum BallColor { PURPLE, GREEN, NONE }
    private BallColor[] slotss = new BallColor[3];

    public static double START_X;
    public static double START_Y;
    public static double START_HEADING_RAD;

    static final double GOAL_X = 140;
    static final double GOAL_Y = 140;

    public static boolean detect = false;

    // ---------------- HARDWARE ----------------
    private Servo turret1, turret2;
    public static double ser = 0.5;
    public static double turretOffsetDeg = -2.0;
    public static double rpmOffset = -50.0;

    private DcMotorEx intake1, intake2;

    // shooter/hood hardware (Limey also gets these internally)
    private DcMotorEx outtake1, outtake2;
    private Servo hood;

    private TeleOp_Helper limey;

    private Servo flick1, flick2, flick3;

    private enum Phase { IDLE, FIRE, GAP }
    private Phase phase = Phase.IDLE;

    private enum FireMode { NORMAL, REMOVE }
    private FireMode fireMode = FireMode.NORMAL;

    private int[] fireOrder = new int[]{1, 2, 3};
    private int fireStep = 0;
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private final ElapsedTime sensorTimer = new ElapsedTime();
    public static int FIRE_MS = 180;
    public static int GAP_MS  = 150;

    private int activeFlicker = -1;

    private boolean lastY = false;
    private boolean lastStart = false;
    private boolean lastG1Left = false;
    private boolean lastG1Right = false;
    private boolean lastG1Up = false;
    private boolean lastG1Down = false;

    private double slowmode = 1;

    private ColorSensor cs1, cs2, cs3, cs4, cs5, cs6;

    // ===== FLICK CONSTANTS =====
    private final double flick1Rest = 0.0, flick1Fire = 0.55;
    private double flick2Rest = 0.98, flick2Fire = 0.43;
    private double flick3Rest = 0.96, flick3Fire = 0.41;

    private final double flick1Fire2 = 0.35;
    private final double flick2Fire2 = 0.63;
    private final double flick3Fire2 = 0.61;

    public static boolean reverseintake = false;

    // ---------------- INIT ----------------
    @Override
    public void init() {

        turret1 = hardwareMap.get(Servo.class, "turret1");
        turret2 = hardwareMap.get(Servo.class, "turret2");

        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        outtake1 = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake2 = hardwareMap.get(DcMotorEx.class, "outtake2");
        outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        hood = hardwareMap.get(Servo.class, "hood");

        flick1 = hardwareMap.get(Servo.class, "flick1");
        flick2 = hardwareMap.get(Servo.class, "flick2");
        flick3 = hardwareMap.get(Servo.class, "flick3");

        cs1 = hardwareMap.get(ColorSensor.class, "cs1");
        cs2 = hardwareMap.get(ColorSensor.class, "cs2");
        cs3 = hardwareMap.get(ColorSensor.class, "cs3");
        cs4 = hardwareMap.get(ColorSensor.class, "cs4");
        cs5 = hardwareMap.get(ColorSensor.class, "cs5");
        cs6 = hardwareMap.get(ColorSensor.class, "cs6");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(START_X, START_Y, START_HEADING_RAD));
        poseHistory = follower.getPoseHistory();

        // ✅ Limey helper
        limey = new TeleOp_Helper(hardwareMap);
        limey.start();

        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        follower.update();
    }

    // ---------------- LOOP ----------------
    @Override
    public void loop() {
        handleDriverTuning();
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
        double turretAngleDeg = Math.toDegrees((goalAngle - pose.getHeading()));

        turretAngleDeg = wrapDegrees(turretAngleDeg);
        double turretDegWithOffset = turretAngleDeg + turretOffsetDeg;

        ser = 0.5 - (turretDegWithOffset / 225.0);
        ser = Range.clip(ser, 0.10, 0.95);

        turret1.setPosition(ser);
        turret2.setPosition(ser);

        // ---------------- SHOOTER (X ON / A OFF) + LIMELIGHT AUTO ----------------
        limey.updateShooterToggle(gamepad1.x, gamepad1.a);
        boolean llUpdated = limey.updateAuto(true);

        // ---------------- INTAKE ----------------
        if (gamepad1.right_bumper || gamepad2.right_bumper) {
            intake1.setPower(-0.9);
            intake2.setPower(0.9);
        } else if (gamepad1.right_trigger > 0.4 || gamepad2.left_bumper) {
            intake1.setPower(0);
            intake2.setPower(0);
        } else if (reverseintake) {
            intake1.setPower(0.5);
            intake2.setPower(-0.5);
        }

        // ---------------- FLICKER FSM ----------------
        boolean y = gamepad1.y;

        if (y && !lastY && phase == Phase.IDLE) {
            fireMode = FireMode.NORMAL;
            fireOrder = buildBestFireOrder(new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.GREEN});

            fireStep = 0;
            activeFlicker = fireOrder[fireStep];
            phase = Phase.FIRE;
            phaseTimer.reset();
        }
        lastY = y;

        boolean startBtn = gamepad2.start;
        if (startBtn && !lastStart && phase == Phase.IDLE) {
            fireMode = FireMode.REMOVE;

            fireOrder = new int[]{1, 2, 3};
            fireStep = 0;
            activeFlicker = fireOrder[fireStep];


            intake1.setPower(1);
            intake2.setPower(-1);
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

        // ---------------- DETECT TOGGLE (LEFT TRIGGER REMOVED) ----------------
        // Only right trigger disables detect
        if (gamepad2.right_trigger > 0.2) {
            detect = false;
        }

        if (detect && sensorTimer.seconds() > 0.2) {
            readSlots();
            sensorTimer.reset();
        }

        // ---------------- TELEMETRY ----------------
        // ONLY update telemetry when Limelight produced a valid read this loop.
        // Show ONLY: target RPM (velocity) and outtake encoder readings.
        if (llUpdated) {
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
    private void fire(int f) {
        if (f < 1 || f > 3) return;

        if (fireMode == FireMode.NORMAL) {
            if (f == 1) flick1.setPosition(flick1Fire);
            else if (f == 2) flick2.setPosition(flick2Fire);
            else flick3.setPosition(flick3Fire);
        } else { // REMOVE
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

    private void readSlots() {
        slotss[0] = detect(cs1, cs2);
        slotss[1] = detect(cs3, cs4);
        slotss[2] = detect(cs5, cs6);
    }

    private BallColor detect(ColorSensor a, ColorSensor b) {
        int r = a.red() + b.red();
        int g = a.green() + b.green();
        int bl = a.blue() + b.blue();
        int alpha = a.alpha() + b.alpha();

        if (alpha < 300) return BallColor.NONE;
        if (r + g + bl < 120) return BallColor.NONE;

        return (g > r && g > bl) ? BallColor.GREEN : BallColor.PURPLE;
    }

    private double wrapDegrees(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
    private void handleDriverTuning() {

        boolean left  = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean up    = gamepad1.dpad_up;
        boolean down  = gamepad1.dpad_down;

        if (right && !lastG1Right) {
            turretOffsetDeg -= 1.0;  // dpad right = minus 1 deg
        }

        if (left && !lastG1Left) {
            turretOffsetDeg += 1.0;  // dpad left = plus 1 deg
        }

        if (up && !lastG1Up) {
            rpmOffset += 25.0;
        }

        if (down && !lastG1Down) {
            rpmOffset -= 25.0;
        }

        lastG1Left = left;
        lastG1Right = right;
        lastG1Up = up;
        lastG1Down = down;
    }

    public int[] buildBestFireOrder(BallColor[] desiredPattern) {
        boolean[] used = new boolean[3];
        int[] fireOrder = new int[3];
        int idx = 0;

        for (BallColor target : desiredPattern) {
            boolean found = false;
            for (int i = 0; i < 3; i++) {
                if (!used[i] && slotss[i] == target) {
                    fireOrder[idx++] = i + 1;
                    used[i] = true;
                    found = true;
                    break;
                }
            }

            if (!found) {
                for (int i = 0; i < 3; i++) {
                    if (!used[i]) {
                        fireOrder[idx++] = i + 1;
                        used[i] = true;
                        break;
                    }
                }
            }
        }
        return fireOrder;
    }
}