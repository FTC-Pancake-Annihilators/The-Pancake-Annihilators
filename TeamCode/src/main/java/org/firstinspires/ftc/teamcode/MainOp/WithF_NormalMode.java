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
    // Introducing all the Hardware
    public DcMotorEx lf, rf, lb, rb, shooter;
    public CRServo leftAdvancer, rightAdvancer;
    public RevBlinkinLedDriver blinkin;//hopefully it works.
    public DistanceSensor distL, distR;
    public VoltageSensor voltSensor;

    // Logic States
    private boolean shooterOn = false, autoFaceOn = false, wasEmpty = false, isAlertingEmpty = false;
    private double targetVelo = 0, pulseEndTime = 0, pulseDir = 0;
    private ElapsedTime emptyTimer = new ElapsedTime(), blinkTimer = new ElapsedTime(), pulseTimer = new ElapsedTime();
    private int blinkCount = 0;
    private boolean lastRB2 = false, lastLB1 = false, lastX2 = false, lastB2 = false;

    // Constants for all
    final double EMPTY_CM = 8.0, minD = 53, maxD = 121, minV = 2400, maxV = 2780;
    private webCamOp camera;

    @Override
    public void init() {
        // Initializing Drive Motors
        lf = hardwareMap.get(DcMotorEx.class, "lf_Drive");
        rf = hardwareMap.get(DcMotorEx.class, "rf_Drive");
        lb = hardwareMap.get(DcMotorEx.class, "lb_Drive");
        rb = hardwareMap.get(DcMotorEx.class, "rb_Drive");

        // Initializing Mechanisms
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        leftAdvancer = hardwareMap.get(CRServo.class, "leftAdvancer");
        rightAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");

        // Initializing Sensors & Indicators
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");//HOPEFULLLLYYYY it works
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
        //make sure to Update the camara
        camera.update();
        AprilTagDetection tag = getTargetTag();
        //run all Voids...
        handleBucketCheck();
        handleDrivetrain(tag);
        handleShooter(tag);
        handleAdvancers();
        updateLEDs(tag);

        //Send time to Driver Hub for testing
        telemetry.addData("Match Time", "%.1f", getRuntime());
        telemetry.update();
    }

    // ----Battery CHECK-----
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

    // --- DRIVING & AUTO-FACE-To-Goal ---
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

    // --- SHOOTER ---
    void handleShooter(AprilTagDetection tag) {
        if (gamepad2.right_bumper && !lastRB2) shooterOn = !shooterOn;
        lastRB2 = gamepad2.right_bumper;

        if (tag != null) {
            // This allows us to to get the Target Velocity by using the AprilTag
            targetVelo = Range.clip(((tag.ftcPose.range - minD)/(maxD - minD)) * (maxV - minV) + minV, minV, maxV);
        } else {
            targetVelo = maxV;// We use maxV because we are comfortable shooting from the Far Zone.
        }
        shooter.setVelocity(shooterOn ? targetVelo : 0);
    }

    // --- FEEDERS ---
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

    // ---LEDS---
    void updateLEDs(AprilTagDetection tag) {
        // PRIORITY 1: this tells us if any of the hardware failed.
        if (Double.isNaN(distL.getDistance(DistanceUnit.CM))) {
            blinkin.setPattern((System.currentTimeMillis() / 200) % 2 == 0 ?
                    RevBlinkinLedDriver.BlinkinPattern.HOT_PINK : RevBlinkinLedDriver.BlinkinPattern.BLACK);
            return;
        }

        double matchTime = getRuntime();

        // PRIORITY 2: Match timer.
        if (matchTime > 115) {
            blinkin.setPattern((System.currentTimeMillis() / 100) % 2 == 0 ?
                    RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.WHITE);
            return;
        }

        // PRIORITY 3: endgame notification the first 10 secs.
        if (matchTime > 90 && matchTime < 100) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            return;
        }

        // PRIORITY 4: 5 blicks to tell us if the bucket is empty.
        if (isAlertingEmpty) {
            if (blinkCount < 10) {
                if (blinkTimer.milliseconds() > 50) { blinkCount++; blinkTimer.reset(); }
                blinkin.setPattern(blinkCount % 2 == 0 ?
                        RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.BLACK);
                return;
            } else { isAlertingEmpty = false; }
        }

        // PRIORITY 5: tell the status of the auto turn if it's taken place.
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

        // PRIORITY 6: tells if the robot is in the allowed range, so that the dirver knows if he can shoot are not.
        if (tag != null && (tag.ftcPose.range < minD || tag.ftcPose.range > maxD)) {
            blinkin.setPattern((System.currentTimeMillis() / 250) % 2 == 0 ?
                    RevBlinkinLedDriver.BlinkinPattern.VIOLET : RevBlinkinLedDriver.BlinkinPattern.BLACK);
            return;
        }

        // PRIORITY 7: This tells us if the shooter is at the right speed whenn the driver turns the shooter on.
        if (shooterOn && Math.abs(shooter.getVelocity() - targetVelo) < 60) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
        } else {

            //normal color which represents our team..
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        }
    }

    private AprilTagDetection getTargetTag() {
        if (camera.seesTag(20)) return camera.getTag(20);
        if (camera.seesTag(24)) return camera.getTag(24);
        return null;
    }
}