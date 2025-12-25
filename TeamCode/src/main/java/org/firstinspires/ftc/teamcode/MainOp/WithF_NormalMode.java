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

@TeleOp(name = "Master_Bot_Official_Clean")
public class WithF_NormalMode extends OpMode {
    public DcMotorEx lf, rf, lb, rb, shooter;
    public CRServo leftAdvancer, rightAdvancer;
    public RevBlinkinLedDriver blinkin;
    public DistanceSensor distL, distR;
    public VoltageSensor voltSensor;

    private boolean shooterOn = false, autoFaceOn = false, wasEmpty = false, isAlertingEmpty = false;
    private double targetVelo = 0, pulseEndTime = 0, pulseDir = 0;
    private ElapsedTime emptyTimer = new ElapsedTime(), blinkTimer = new ElapsedTime(), pulseTimer = new ElapsedTime();
    private int blinkCount = 0;
    private boolean lastRB2 = false, lastLB1 = false, lastX2 = false, lastB2 = false;

    final double EMPTY_CM = 8.0, minD = 53, maxD = 121, minV = 2400, maxV = 2780;
    private webCamOp camera;

    @Override
    public void init() {
        lf = hardwareMap.get(DcMotorEx.class, "lf_Drive");
        rf = hardwareMap.get(DcMotorEx.class, "rf_Drive");
        lb = hardwareMap.get(DcMotorEx.class, "lb_Drive");
        rb = hardwareMap.get(DcMotorEx.class, "rb_Drive");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        leftAdvancer = hardwareMap.get(CRServo.class, "leftAdvancer");
        rightAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        distL = hardwareMap.get(DistanceSensor.class, "distLeft");
        distR = hardwareMap.get(DistanceSensor.class, "distRight");
        voltSensor = hardwareMap.voltageSensor.iterator().next();
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        camera = new webCamOp(hardwareMap);
    }

    @Override
    public void init_loop() { handleBatteryMonitor(); }

    @Override
    public void start() { resetRuntime(); }

    @Override
    public void loop() {
        camera.update();
        AprilTagDetection tag = getTargetTag();

        handleBucketCheck();
        handleDrivetrain(tag);
        handleShooter(tag);
        handleAdvancers();
        updateLEDs(tag);
    }

    // --- LOGIC VOIDS ---

    void handleBatteryMonitor() {
        double v = voltSensor.getVoltage();
        if (v > 13.5) blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        else if (v > 12.5) blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        else blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }

    void handleBucketCheck() {
        boolean empty = (distL.getDistance(DistanceUnit.CM) > EMPTY_CM && distR.getDistance(DistanceUnit.CM) > EMPTY_CM);
        if (empty) {
            if (!wasEmpty) { emptyTimer.reset(); wasEmpty = true; }
            if (emptyTimer.milliseconds() > 500 && !isAlertingEmpty) { isAlertingEmpty = true; blinkCount = 0; blinkTimer.reset(); }
        } else wasEmpty = false;
    }

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

    void handleShooter(AprilTagDetection tag) {
        if (gamepad2.right_bumper && !lastRB2) shooterOn = !shooterOn;
        lastRB2 = gamepad2.right_bumper;
        if (tag != null) targetVelo = Range.clip(((tag.ftcPose.range - minD)/(maxD - minD)) * (maxV - minV) + minV, minV, maxV);
        else targetVelo = maxV;
        shooter.setVelocity(shooterOn ? targetVelo : 0);
    }

    void handleAdvancers() {
        if (gamepad2.x && !lastX2) { pulseDir = 1.0; pulseEndTime = pulseTimer.milliseconds() + 50; }
        else if (gamepad2.b && !lastB2) { pulseDir = -1.0; pulseEndTime = pulseTimer.milliseconds() + 50; }
        lastX2 = gamepad2.x; lastB2 = gamepad2.b;
        if (pulseTimer.milliseconds() < pulseEndTime) { leftAdvancer.setPower(pulseDir); rightAdvancer.setPower(pulseDir); }
        else { leftAdvancer.setPower(0); rightAdvancer.setPower(0); }
    }

    void updateLEDs(AprilTagDetection tag) {
        // 1. HARDWARE FAIL (Priority 1)
        if (Double.isNaN(distL.getDistance(DistanceUnit.CM))) {
            blinkin.setPattern((System.currentTimeMillis() / 200) % 2 == 0 ?
                    RevBlinkinLedDriver.BlinkinPattern.HOT_PINK : RevBlinkinLedDriver.BlinkinPattern.BLACK);
            return;
        }

        // 2. MATCH TIMER (Priority 2)
        double matchTime = getRuntime();
        if (matchTime > 115) { // Last 5 Seconds (Red/White Strobe)
            blinkin.setPattern((System.currentTimeMillis() / 100) % 2 == 0 ?
                    RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.WHITE);
            return;
        }
        else if (matchTime > 90) { // Last 30 Seconds (Solid Gold)
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
            return;
        }

        // 3. EMPTY BUCKET (Priority 3 - Blinks 5 Times)
        if (isAlertingEmpty) {
            if (blinkCount < 10) {
                if (blinkTimer.milliseconds() > 50) {
                    blinkCount++;
                    blinkTimer.reset();
                }
                blinkin.setPattern(blinkCount % 2 == 0 ?
                        RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.BLACK);
                return;
            } else {
                isAlertingEmpty = false;
            }
        }

        // 4. AUTO-FACE LOCK (Priority 4)
        if (autoFaceOn) {
            if (tag == null) { // Searching (Blue/Black slow blink)
                blinkin.setPattern((System.currentTimeMillis() / 300) % 2 == 0 ?
                        RevBlinkinLedDriver.BlinkinPattern.BLUE : RevBlinkinLedDriver.BlinkinPattern.BLACK);
            } else if (Math.abs(tag.ftcPose.bearing) > 5) { // Partial (Strobe Blue)
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
            } else { // Perfect (Solid Blue)
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }
            return;
        }

        // 5. OUT OF RANGE (Priority 5)
        if (tag != null && (tag.ftcPose.range < minD || tag.ftcPose.range > maxD)) {
            blinkin.setPattern((System.currentTimeMillis() / 250) % 2 == 0 ?
                    RevBlinkinLedDriver.BlinkinPattern.VIOLET : RevBlinkinLedDriver.BlinkinPattern.BLACK);
            return;
        }

        // 6. SHOOTER READY & IDLE (Lowest Priority)
        if (shooterOn && Math.abs(shooter.getVelocity() - targetVelo) < 60) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE); // Green Pulse
        } else {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED); // Solid Red Idle
        }
    }

    private AprilTagDetection getTargetTag() {
        if (camera.seesTag(20)) return camera.getTag(20);
        if (camera.seesTag(24)) return camera.getTag(24);
        return null;
    }
}