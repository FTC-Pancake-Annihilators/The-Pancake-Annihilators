package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


public class With_LED extends OpMode {
    public DcMotorEx lf_Drive, lb_Drive, rf_Drive, rb_Drive;
    public DcMotorEx shooter;
    public CRServo leftAdvancer, rightAdvancer;
    public RevBlinkinLedDriver blinkin;

    private boolean shooterMoving = false;
    private boolean autoFaceActive = false;
    private ElapsedTime pulseTimer = new ElapsedTime();
    private double pulseEndTime = 0;
    private double pulseDirection = 0;

    private boolean lastRightBumper2 = false, lastLeftBumper1 = false;
    private boolean lastX2 = false, lastB2 = false;
    private double targetShooterVelocity = 0;

//    final double minDis = 53, maxDis = 121, minVelo = 2400, maxVelo = 2800;
    final double minDis = 40, maxDis = 130, minVelo=1900,maxVelo=2500;
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
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        lf_Drive.setDirection(DcMotorEx.Direction.REVERSE);
        lb_Drive.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        camera = new webCamOp(hardwareMap);
    }

    @Override
    public void loop() {
        camera.update();

        if (gamepad1.left_bumper && !lastLeftBumper1) autoFaceActive = !autoFaceActive;
        lastLeftBumper1 = gamepad1.left_bumper;
        autoFaceControl(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);

        calculateShooterVelocity();
        if (gamepad2.right_bumper && !lastRightBumper2) shooterMoving = !shooterMoving;
        lastRightBumper2 = gamepad2.right_bumper;

        if (shooterMoving) shooter.setVelocity(targetShooterVelocity);
        else shooter.setVelocity(0);

        handleTimedAdvancers();
        updateLEDs();
        telemetry.addData("ActualVelo", shooter.getVelocity());

    }

    public void updateLEDs() {
        boolean shooterReady = shooterMoving && (Math.abs(shooter.getVelocity() - targetShooterVelocity) < 60);
        boolean faceReady = autoFaceActive && (getTargetTag() != null);

        if (shooterReady && faceReady) {
            // This is the official name for the green pulsing/heartbeat effect
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
        } else if (shooterReady) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        } else if (faceReady) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        } else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }

    public void handleTimedAdvancers() {
        if (gamepad2.x && !lastX2) { pulseDirection = 1.0; pulseEndTime = pulseTimer.milliseconds() + 50; }
        else if (gamepad2.b && !lastB2) { pulseDirection = -1.0; pulseEndTime = pulseTimer.milliseconds() + 50; }
        lastX2 = gamepad2.x; lastB2 = gamepad2.b;

        if (pulseTimer.milliseconds() < pulseEndTime) {
            leftAdvancer.setPower(pulseDirection);
            rightAdvancer.setPower(pulseDirection);
        } else {
            leftAdvancer.setPower(0);
            rightAdvancer.setPower(0);
            pulseDirection = 0;
        }
    }

    public void autoFaceControl(double fwd, double str, double rot) {
        double turn = rot;
        if (autoFaceActive) {
            AprilTagDetection tag = getTargetTag();
            if (tag != null) turn = Range.clip(-tag.ftcPose.bearing * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        }
        mecanumDrive(fwd, str, turn);
    }

    public void calculateShooterVelocity() {
        AprilTagDetection tag = getTargetTag();
        if (tag != null) {
            double d = tag.ftcPose.range;
            targetShooterVelocity = Range.clip(((d - minDis)/(maxDis - minDis)) * (maxVelo - minVelo) + minVelo, minVelo, maxVelo);
        } else targetShooterVelocity = maxVelo;
    }

    private AprilTagDetection getTargetTag() {
        if (camera.seesTag(20)) return camera.getTag(20);
        if (camera.seesTag(24)) return camera.getTag(24);
        return null;
    }

    public void mecanumDrive(double fwd, double str, double rot) {
        double d = Math.max(Math.abs(fwd) + Math.abs(str) + Math.abs(rot), 1);
        lf_Drive.setPower((fwd + str + rot) / d);
        rf_Drive.setPower((fwd - str - rot) / d);
        lb_Drive.setPower((fwd - str + rot) / d);
        rb_Drive.setPower((fwd + str - rot) / d);
    }
}