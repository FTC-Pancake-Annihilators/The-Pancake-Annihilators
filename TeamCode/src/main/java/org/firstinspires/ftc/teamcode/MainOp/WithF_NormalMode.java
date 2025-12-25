package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Bot_2_Official_Full")
public class WithF_NormalMode extends OpMode {
    public DcMotorEx lf_Drive, lb_Drive, rf_Drive, rb_Drive, shooter;
    public CRServo leftAdvancer, rightAdvancer;
    public RevBlinkinLedDriver blinkin;
    public DistanceSensor distLeft, distRight;

    private boolean shooterMoving = false, autoFaceActive = false;
    private double targetShooterVelocity = 0;
    private ElapsedTime pulseTimer = new ElapsedTime();
    private double pulseEndTime = 0, pulseDirection = 0;

    private ElapsedTime emptyTimer = new ElapsedTime();
    private boolean bucketWasEmpty = false;
    private boolean isAlertingEmpty = false;
    private int blinkCount = 0;
    private ElapsedTime blinkIntervalTimer = new ElapsedTime();

    private boolean lastRightBumper2 = false, lastLeftBumper1 = false, lastX2 = false, lastB2 = false;

    final double EMPTY_LIMIT = 8.0;
    final double minDis = 53, maxDis = 121, minVelo = 2400, maxVelo = 2780;

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
        distLeft = hardwareMap.get(DistanceSensor.class, "distLeft");
        distRight = hardwareMap.get(DistanceSensor.class, "distRight");

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        camera = new webCamOp(hardwareMap);
    }

    @Override
    public void loop() {
        camera.update();

        boolean currentlyEmpty = (distLeft.getDistance(DistanceUnit.CM) > EMPTY_LIMIT &&
                distRight.getDistance(DistanceUnit.CM) > EMPTY_LIMIT);
        if (currentlyEmpty) {
            if (!bucketWasEmpty) { emptyTimer.reset(); bucketWasEmpty = true; }
            if (emptyTimer.milliseconds() > 500 && !isAlertingEmpty) {
                isAlertingEmpty = true; blinkCount = 0; blinkIntervalTimer.reset();
            }
        } else { bucketWasEmpty = false; }

        if (gamepad1.left_bumper && !lastLeftBumper1) autoFaceActive = !autoFaceActive;
        lastLeftBumper1 = gamepad1.left_bumper;
        autoFaceControl(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);

        calculateShooterVelocity();
        if (gamepad2.right_bumper && !lastRightBumper2) shooterMoving = !shooterMoving;
        lastRightBumper2 = gamepad2.right_bumper;
        if (shooterMoving) shooter.setVelocity(targetShooterVelocity); else shooter.setVelocity(0);

        handleTimedAdvancers();
        updateLEDs();
    }

    public void updateLEDs() {
        // 1. URGENT EMPTY STROBE
        if (isAlertingEmpty) {
            if (blinkCount < 10) {
                if (blinkIntervalTimer.milliseconds() > 50) { blinkCount++; blinkIntervalTimer.reset(); }
                blinkin.setPattern(blinkCount % 2 == 0 ?
                        RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.BLACK);
                return;
            } else { isAlertingEmpty = false; }
        }

        AprilTagDetection tag = getTargetTag();

        // 2. OUT OF RANGE ALERT
        if (tag != null && (tag.ftcPose.range < minDis || tag.ftcPose.range > maxDis)) {
            // Violet blink logic using system time
            if ((System.currentTimeMillis() / 250) % 2 == 0) {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
            } else {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            }
            return;
        }

        // 3. COMBAT STATUS
        boolean shooterReady = shooterMoving && (Math.abs(shooter.getVelocity() - targetShooterVelocity) < 60);
        boolean faceReady = autoFaceActive && (tag != null);

        if (shooterReady && faceReady) blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
        else if (shooterReady) blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        else if (faceReady) blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        else blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
    }

    // handleTimedAdvancers, autoFaceControl, calculateShooterVelocity, getTargetTag are identical to Servo version...
    public void handleTimedAdvancers() {
        if (gamepad2.x && !lastX2) { pulseDirection = 1.0; pulseEndTime = pulseTimer.milliseconds() + 50; }
        else if (gamepad2.b && !lastB2) { pulseDirection = -1.0; pulseEndTime = pulseTimer.milliseconds() + 50; }
        lastX2 = gamepad2.x; lastB2 = gamepad2.b;
        if (pulseTimer.milliseconds() < pulseEndTime) {
            leftAdvancer.setPower(pulseDirection); rightAdvancer.setPower(pulseDirection);
        } else { leftAdvancer.setPower(0); rightAdvancer.setPower(0); pulseDirection = 0; }
    }

    public void autoFaceControl(double f, double s, double r) {
        double turn = r;
        if (autoFaceActive) {
            AprilTagDetection tag = getTargetTag();
            if (tag != null) turn = Range.clip(-tag.ftcPose.bearing * 0.02, -0.4, 0.4);
        }
        double d = Math.max(Math.abs(f) + Math.abs(s) + Math.abs(turn), 1);
        lf_Drive.setPower((f + s + turn) / d); rf_Drive.setPower((f - s - turn) / d);
        lb_Drive.setPower((f - s + turn) / d); rb_Drive.setPower((f + s - turn) / d);
    }

    public void calculateShooterVelocity() {
        AprilTagDetection tag = getTargetTag();
        if (tag != null) targetShooterVelocity = Range.clip(((tag.ftcPose.range - minDis)/(maxDis - minDis)) * (maxVelo - minVelo) + minVelo, minVelo, maxVelo);
        else targetShooterVelocity = maxVelo;
    }

    private AprilTagDetection getTargetTag() {
        if (camera.seesTag(20)) return camera.getTag(20);
        if (camera.seesTag(24)) return camera.getTag(24);
        return null;
    }
}