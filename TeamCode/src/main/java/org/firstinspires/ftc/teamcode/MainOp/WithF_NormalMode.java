package org.firstinspires.ftc.teamcode.MainOp;

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
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver; // Import the driver
import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.Artboard;

@TeleOp(name = "Master_Bot_Prism_Official")
public class WithF_NormalMode extends OpMode {
    // Hardware
    public DcMotorEx lf, rf, lb, rb, shooter;
    public CRServo leftAdvancer, rightAdvancer;
    public GoBildaPrismDriver prism; // Official Driver class
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
        lf = hardwareMap.get(DcMotorEx.class, "lf_Drive");
        rf = hardwareMap.get(DcMotorEx.class, "rf_Drive");
        lb = hardwareMap.get(DcMotorEx.class, "lb_Drive");
        rb = hardwareMap.get(DcMotorEx.class, "rb_Drive");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        leftAdvancer = hardwareMap.get(CRServo.class, "leftAdvancer");
        rightAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");

        // Initialize Prism Driver
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");

        distL = hardwareMap.get(DistanceSensor.class, "distLeft");
        distR = hardwareMap.get(DistanceSensor.class, "distRight");
        voltSensor = hardwareMap.voltageSensor.iterator().next();

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        camera = new webCamOp(hardwareMap);
    }

    @Override
    public void init_loop() {
        // Battery check on initialization
        double v = voltSensor.getVoltage();
        if (v > 13.0) prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_0); // Healthy (Green)
        else prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_5); // Low (Red)
    }

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

        telemetry.addData("Match Time", "%.1f", getRuntime());
        telemetry.update();
    }

    // --- LOGIC FUNCTIONS ---

    void handleBucketCheck() {
        boolean empty = (distL.getDistance(DistanceUnit.CM) > EMPTY_CM && distR.getDistance(DistanceUnit.CM) > EMPTY_CM);
        if (empty) {
            if (!wasEmpty) { emptyTimer.reset(); wasEmpty = true; }
            if (emptyTimer.milliseconds() > 500 && !isAlertingEmpty) {
                isAlertingEmpty = true;
                blinkCount = 0;
                blinkTimer.reset();
            }
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

    // --- PRISM LED PRIORITY SYSTEM ---
    void updateLEDs(AprilTagDetection tag) {
        // Priority 1: Hardware Fail
        if (Double.isNaN(distL.getDistance(DistanceUnit.CM))) {
            prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_4); // Set this to Pink Strobe
            return;
        }

        double matchTime = getRuntime();

        // Priority 2: Match Warning (Last 5s)
        if (matchTime > 115) {
            prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_5); // Set this to Red/White Strobe
            return;
        }

        // Priority 3: Endgame (90s)
        if (matchTime > 90 && matchTime < 100) {
            prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_2); // Set this to Gold
            return;
        }

        // Priority 4: Empty Bucket Alert
        if (isAlertingEmpty) {
            prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_5); // Trigger Red Alert
            // After 5 blinks logic can be added here or just let the user see it
            if (blinkTimer.milliseconds() > 1000) isAlertingEmpty = false;
            return;
        }

        // Priority 5: Auto-Face Lock
        if (autoFaceOn) {
            if (tag == null) prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_3); // Searching Blue
            else prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_1); // Locked Blue
            return;
        }

        // Priority 6: Out of Range
        if (tag != null && (tag.ftcPose.range < minD || tag.ftcPose.range > maxD)) {
            prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_4); // Purple/Violet
            return;
        }

        // Priority 7: Shooter Ready
        if (shooterOn && Math.abs(shooter.getVelocity() - targetVelo) < 60) {
            prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_0); // Green
        } else {
            prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_2); // Orange/Team Color
        }
    }

    private AprilTagDetection getTargetTag() {
        if (camera.seesTag(20)) return camera.getTag(20);
        if (camera.seesTag(24)) return camera.getTag(24);
        return null;
    }
}