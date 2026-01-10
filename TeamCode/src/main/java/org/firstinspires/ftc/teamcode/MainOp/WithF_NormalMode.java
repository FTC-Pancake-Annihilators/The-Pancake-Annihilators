package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "With LEDS", group = "AAAA")
public class WithF_NormalMode extends OpMode {
    // Hardware
    public DcMotorEx lf, rf, lb, rb, shooter;
    public CRServo leftAdvancer, rightAdvancer;
    public Servo prismLED; // PWM mode - name "prismLED" in config
    public VoltageSensor voltSensor;

    // Logic States
    private boolean shooterOn = false, autoFaceOn = false;
    private double targetVelo = 0, pulseEndTime = 0, pulseDir = 0;
    private ElapsedTime pulseTimer = new ElapsedTime();
    private boolean lastRB2 = false, lastLB1 = false, lastX2 = false, lastB2 = false;

    // Constants
    final double minD = 53, maxD = 121, minV = 2400, maxV = 2780;
    private webCamOp camera;

    // PWM positions (from manual table - test & adjust)
    private static final double HEALTHY_GREEN = 0.35;
    private static final double LOW_BATTERY_RED = 0.55;
    private static final double ENDGAME_WARNING = 0.295;  // Emergency Lights flash
    private static final double ENDGAME_GOLD = 0.30;      // Orange/gold
    private static final double AUTO_FACE_SEARCH = 0.92;  // Blue solid
    private static final double AUTO_FACE_LOCKED = 0.92;  // Blue solid
    private static final double OUT_OF_RANGE = 0.60;      // Purple
    private static final double SHOOTER_READY = 0.35;     // Green pulse
    private static final double DEFAULT_ORANGE = 0.55;    // Orange idle

    @Override
    public void init() {
        lf = hardwareMap.get(DcMotorEx.class, "lf_Drive");
        rf = hardwareMap.get(DcMotorEx.class, "rf_Drive");
        lb = hardwareMap.get(DcMotorEx.class, "lb_Drive");
        rb = hardwareMap.get(DcMotorEx.class, "rb_Drive");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        leftAdvancer = hardwareMap.get(CRServo.class, "leftAdvancer");
        rightAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");

        prismLED = hardwareMap.get(Servo.class, "prismLED");

        voltSensor = hardwareMap.voltageSensor.iterator().next();

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        camera = new webCamOp(hardwareMap);
    }

    @Override
    public void init_loop() {
        double v = voltSensor.getVoltage();
        if (v > 13.0) prismLED.setPosition(HEALTHY_GREEN);
        else prismLED.setPosition(LOW_BATTERY_RED);
    }

    @Override
    public void start() { resetRuntime(); }

    @Override
    public void loop() {
        camera.update();
        AprilTagDetection tag = getTargetTag();

        handleDrivetrain(tag);
        handleShooter(tag);
        handleAdvancers();
        updateLEDs(tag);

        telemetry.addData("Match Time", "%.1f", getRuntime());
        telemetry.update();
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

    // PWM LED Priority System (no hardware fail or bucket check)
    void updateLEDs(AprilTagDetection tag) {
        double matchTime = getRuntime();

        // Priority 1: Last 5s warning
        if (matchTime > 115) {
            prismLED.setPosition(ENDGAME_WARNING); // Emergency Lights flash
            return;
        }

        // Priority 2: Endgame (90-100s)
        if (matchTime > 90 && matchTime < 100) {
            prismLED.setPosition(ENDGAME_GOLD); // Orange/gold
            return;
        }

        // Priority 3: Auto-Face
        if (autoFaceOn) {
            if (tag == null) prismLED.setPosition(AUTO_FACE_SEARCH); // Blue solid
            else prismLED.setPosition(AUTO_FACE_LOCKED); // Blue solid
            return;
        }

        // Priority 4: Out of Range
        if (tag != null && (tag.ftcPose.range < minD || tag.ftcPose.range > maxD)) {
            prismLED.setPosition(OUT_OF_RANGE); // Purple
            return;
        }

        // Priority 5: Shooter Ready
        if (shooterOn && Math.abs(shooter.getVelocity() - targetVelo) < 60) {
            prismLED.setPosition(SHOOTER_READY); // Green pulse
        } else {
            prismLED.setPosition(DEFAULT_ORANGE); // Orange idle
        }
        // Priority 5: Shooter Ready
            prismLED.setPosition(SHOOTER_READY); // Green pulse
            prismLED.setPosition(DEFAULT_ORANGE); // Orange idle
    }

    private AprilTagDetection getTargetTag() {
        if (camera.seesTag(20)) return camera.getTag(20);
        if (camera.seesTag(24)) return camera.getTag(24);
        return null;
    }
}