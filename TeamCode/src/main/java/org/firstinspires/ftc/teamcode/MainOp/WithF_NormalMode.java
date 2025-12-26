package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Master_Bot_Final_Release")
public class WithF_NormalMode extends OpMode {
    // Hardware
    public DcMotorEx lf, rf, lb, rb, shooter;
    public CRServo leftAdvancer, rightAdvancer;
    public RevBlinkinLedDriver blinkin;
    public DistanceSensor distL, distR;
    public VoltageSensor voltSensor;

    // Logic States
    private boolean shooterOn = false, autoFaceOn = false, wasEmpty = false, isAlertingEmpty = false;
    private double targetVelo = 0, pulseEndTime = 0, pulseDir = 0;
    private ElapsedTime emptyTimer = new ElapsedTime(), blinkTimer = new ElapsedTime(), pulseTimer = new ElapsedTime();
    private int blinkCount = 0;
    private boolean lastRB2 = false, lastLB1 = false, lastX2 = false, lastB2 = false;

    // Constants
    final double EMPTY_CM = 8.0, minD = 53, maxD = 121, minV = 2400, maxV = 2780;
    private webCamOp camera;

    @Override
    public void init() {
        // Drive Motors
        lf = hardwareMap.get(DcMotorEx.class, "lf_Drive");
        rf = hardwareMap.get(DcMotorEx.class, "rf_Drive");
        lb = hardwareMap.get(DcMotorEx.class, "lb_Drive");
        rb = hardwareMap.get(DcMotorEx.class, "rb_Drive");

        // Mechanisms
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        leftAdvancer = hardwareMap.get(CRServo.class, "leftAdvancer");
        rightAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");

        // Sensors & Indicators
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        distL = hardwareMap.get(DistanceSensor.class, "distLeft");
        distR = hardwareMap.get(DistanceSensor.class, "distRight");
        voltSensor = hardwareMap.voltageSensor.iterator().next();

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        camera = new webCamOp(hardwareMap);
    }

    @Override
    public void init_loop() {
        handleBatteryMonitor(); // Pre-match health check
    }

    @Override
    public void start() {
        resetRuntime(); // Ensure match clock starts at 0
    }

    @Override
    public void loop() {
        camera.update();
        AprilTagDetection tag = getTargetTag();

        handleBucketCheck();
        handleDrivetrain(tag);
        handleShooter(tag);
        handleAdvancers();
        updateLEDs(tag);

        // Optional: Send time to Driver Hub for testing
        telemetry.addData("Match Time", "%.1f", getRuntime());
        telemetry.update();
    }

    // --- BATTERY CHECK (RUNS IN INIT) ---
    void handleBatteryMonitor() {
        double v = voltSensor.getVoltage();
        if (v > 13.5) blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        else if (v > 12.5) blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        else blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    // --- BUCKET SENSORS ---
    void handleBucketCheck() {
        boolean empty = (distL.getDistance(DistanceUnit.CM) > EMPTY_CM && distR.getDistance(DistanceUnit.CM) > EMPTY_CM);
        if (empty) {
            if (!wasEmpty) { emptyTimer.reset(); wasEmpty = true; }
            // If empty for 0.5s, trigger the 5-blink alert
            if (emptyTimer.milliseconds() > 500 && !isAlertingEmpty) {
                isAlertingEmpty = true;
                blinkCount = 0;
                blinkTimer.reset();
            }
        } else wasEmpty = false;
    }

    // --- DRIVING & AUTO-FACE ---
    void handleDrivetrain(AprilTagDetection tag) {
        if (gamepad1.left_bumper && !lastLB1) autoFaceOn = !autoFaceOn;
        lastLB1 = gamepad1.left_bumper;

        double turn = gamepad1.right_stick_x;
        if (autoFaceOn && tag != null) turn = Range.clip(-tag.ftcPose.bearing * 0.02, -0.4, 0.4);

        double f = gamepad1.left_stick_y, s = -gamepad1.left_stick_x;
        double d = Math.max(Math.abs(f) + Math.abs(s) + Math.abs(turn), 1);
        lf.setPower((f + s + turn) / d); rf.setPower((f - s - turn) / d);
        lb.setPower((f - s + turn) / d); rb.setPower((f + s - turn) / d);
    }

    // --- SHOOTER PHYSICS ---
    void handleShooter(AprilTagDetection tag) {
        if (gamepad2.right_bumper && !lastRB2) shooterOn = !shooterOn;
        lastRB2 = gamepad2.right_bumper;

        if (tag != null) {
            // LERP math for dynamic RPM based on distance
            targetVelo = Range.clip(((tag.ftcPose.range - minD)/(maxD - minD)) * (maxV - minV) + minV, minV, maxV);
        } else {
            targetVelo = maxV;
        }
        shooter.setVelocity(shooterOn ? targetVelo : 0);
    }

    // --- SERVO ADVANCERS ---
    void handleAdvancers() {
        if (gamepad2.x && !lastX2) { pulseDir = 1.0; pulseEndTime = pulseTimer.milliseconds() + 50; }
        else if (gamepad2.b && !lastB2) { pulseDir = -1.0; pulseEndTime = pulseTimer.milliseconds() + 50; }
        lastX2 = gamepad2.x; lastB2 = gamepad2.b;

        if (pulseTimer.milliseconds() < pulseEndTime) {
            leftAdvancer.setPower(pulseDir); rightAdvancer.setPower(pulseDir);
        } else {
            leftAdvancer.setPower(0); rightAdvancer.setPower(0);
        }
    }

    // --- LED MASTER CONTROL (PRIORITY SYSTEM) ---
    void updateLEDs(AprilTagDetection tag) {
        // PRIORITY 1: HARDWARE FAIL
        if (Double.isNaN(distL.getDistance(DistanceUnit.CM))) {
            blinkin.setPattern((System.currentTimeMillis() / 200) % 2 == 0 ?
                    RevBlinkinLedDriver.BlinkinPattern.HOT_PINK : RevBlinkinLedDriver.BlinkinPattern.BLACK);
            return;
        }

        double matchTime = getRuntime();

        // PRIORITY 2: MATCH TIMER (THE PANIC ZONE)
        if (matchTime > 115) {
            blinkin.setPattern((System.currentTimeMillis() / 100) % 2 == 0 ?
                    RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.WHITE);
            return;
        }

        // PRIORITY 3: ENDGAME NOTIFICATION (90s to 100s Window)
        if (matchTime > 90 && matchTime < 100) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            return;
        }

        // PRIORITY 4: EMPTY BUCKET ALERT (5 Blinks)
        if (isAlertingEmpty) {
            if (blinkCount < 10) {
                if (blinkTimer.milliseconds() > 50) { blinkCount++; blinkTimer.reset(); }
                blinkin.setPattern(blinkCount % 2 == 0 ?
                        RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.BLACK);
                return;
            } else { isAlertingEmpty = false; }
        }

        // PRIORITY 5: TARGETING FEEDBACK
        if (autoFaceOn) {
            if (tag == null) {
                blinkin.setPattern((System.currentTimeMillis() / 300) % 2 == 0 ?
                        RevBlinkinLedDriver.BlinkinPattern.BLUE : RevBlinkinLedDriver.BlinkinPattern.BLACK);
            } else if (Math.abs(tag.ftcPose.bearing) > 5) {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
            } else {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }
            return;
        }

        // PRIORITY 6: RANGE CHECK
        if (tag != null && (tag.ftcPose.range < minD || tag.ftcPose.range > maxD)) {
            blinkin.setPattern((System.currentTimeMillis() / 250) % 2 == 0 ?
                    RevBlinkinLedDriver.BlinkinPattern.VIOLET : RevBlinkinLedDriver.BlinkinPattern.BLACK);
            return;
        }

        // PRIORITY 7: READY & IDLE
        if (shooterOn && Math.abs(shooter.getVelocity() - targetVelo) < 60) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
        } else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
    }

    private AprilTagDetection getTargetTag() {
        if (camera.seesTag(20)) return camera.getTag(20);
        if (camera.seesTag(24)) return camera.getTag(24);
        return null;
    }
}