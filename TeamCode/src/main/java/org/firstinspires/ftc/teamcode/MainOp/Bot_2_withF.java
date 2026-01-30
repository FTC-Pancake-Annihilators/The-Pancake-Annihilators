package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Camera_Speed_Align", group = "B")
public class Bot_2_withF extends OpMode {
    public DcMotorEx lf_Drive, lb_Drive, rf_Drive, rb_Drive;
    public DcMotorEx shooter;
    public CRServo leftAdvancer, rightAdvancer;

    // Toggle and Pulse States
    private boolean shooterMoving = false;
    private boolean autoFaceActive = false;

    // Timed Pulse Variables
    private ElapsedTime pulseTimer = new ElapsedTime();
    private double pulseEndTime = 0;
    private double pulseDirection = 0; // 1 for fwd, -1 for bwd, 0 for off

    // "Last State" trackers
    private boolean lastRightBumper2 = false;
    private boolean lastLeftBumper1 = false;
    private boolean lastX2 = false;
    private boolean lastB2 = false;

    private double targetShooterVelocity = 0;

    // Tuning
    final double minDis = 53, maxDis = 114;
    final double minVelo = 1600, maxVelo = 2025;     // max shooter velocity
    final double TURN_GAIN = 0.02, MAX_AUTO_TURN = 0.4;


    private webCamOp camera;

    @Override
    public void init() {
        lf_Drive = hardwareMap.get(DcMotorEx.class, "lf_Drive");
        rf_Drive = hardwareMap.get(DcMotorEx.class, "rf_Drive");
        lb_Drive = hardwareMap.get(DcMotorEx.class, "lb_Drive");
        rb_Drive = hardwareMap.get(DcMotorEx.class, "rb_Drive");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        leftAdvancer = hardwareMap.get(CRServo.class, "leftAdvancer");
        rightAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");

        lf_Drive.setDirection(DcMotorEx.Direction.REVERSE);
        lb_Drive.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);

        lf_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        camera = new webCamOp(hardwareMap);
        pulseTimer.reset();
    }

    @Override
    public void loop() {
        camera.update();

        // 1. AUTO-FACE TOGGLE (Gamepad 1)
        if (gamepad1.left_bumper && !lastLeftBumper1) {
            autoFaceActive = !autoFaceActive;
        }
        lastLeftBumper1 = gamepad1.left_bumper;
        autoFaceControl(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);

        // 2. SHOOTER TOGGLE (Gamepad 2)
        calculateShooterVelocity();
        if (gamepad2.right_bumper && !lastRightBumper2) {
            shooterMoving = !shooterMoving;
        }
        lastRightBumper2 = gamepad2.right_bumper;

        if (shooterMoving) shooter.setVelocity(targetShooterVelocity);
        else shooter.setVelocity(0);

        // 3. ADVANCER 50MS PULSE (Gamepad 2)
        handleTimedAdvancers();

        // Telemetry
        telemetry.addData("AUTO-FACE", autoFaceActive ? "LOCKED ON" : "MANUAL");
        telemetry.addData("SHOOTER", shooterMoving ? "ON" : "OFF");
        telemetry.addData("PULSE", pulseDirection != 0 ? "RUNNING" : "READY");
        telemetry.update();
    }

    /**
     * VOID: Handles the 50ms pulse logic for X and B buttons.
     */
    public void handleTimedAdvancers() {
        // Detect "Click" for X (Forward)
        if (gamepad2.x && !lastX2) {
            pulseDirection = 1.0;
            pulseEndTime = pulseTimer.milliseconds() + 50; // Set stop time 50ms from now
        }
        // Detect "Click" for B (Reverse)
        else if (gamepad2.b && !lastB2) {
            pulseDirection = -1.0;
            pulseEndTime = pulseTimer.milliseconds() + 50;
        }

        lastX2 = gamepad2.x;
        lastB2 = gamepad2.b;

        // If current time is less than end time, keep running. Otherwise, STOP.
        if (pulseTimer.milliseconds() < pulseEndTime) {
            leftAdvancer.setPower(pulseDirection);
            rightAdvancer.setPower(pulseDirection);
        } else {
            leftAdvancer.setPower(0);
            rightAdvancer.setPower(0);
            pulseDirection = 0; // Reset direction
        }
    }

    public void autoFaceControl(double forward, double strafe, double manualTurn) {
        double finalTurn = manualTurn;
        if (autoFaceActive) {
            AprilTagDetection target = getTargetTag();
            if (target != null) {
                double headingError = target.ftcPose.bearing;
                finalTurn = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            }
        }
        mecanumDrive(forward, strafe, finalTurn);
    }

    public void calculateShooterVelocity() {
        AprilTagDetection target = getTargetTag();
        if (target != null) {
            double distance = target.ftcPose.range;
            double speed = ((distance - minDis) / (maxDis - minDis)) * (maxVelo - minVelo) + minVelo;
            targetShooterVelocity = Range.clip(speed, minVelo, maxVelo);
        } else {
            targetShooterVelocity = maxVelo;
        }
    }

    private AprilTagDetection getTargetTag() {
        if (camera.seesTag(20)) return camera.getTag(20);
        if (camera.seesTag(24)) return camera.getTag(24);
        return null;
    }

    public void mecanumDrive(double forward, double strafe, double rotate) {
        double denom = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        lf_Drive.setPower((forward + strafe + rotate) / denom);
        rf_Drive.setPower((forward - strafe - rotate) / denom);
        lb_Drive.setPower((forward - strafe + rotate) / denom);
        rb_Drive.setPower((forward + strafe - rotate) / denom);
    }
}