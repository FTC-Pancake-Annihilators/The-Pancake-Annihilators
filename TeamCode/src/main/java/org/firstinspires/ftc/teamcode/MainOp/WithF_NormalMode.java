package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@TeleOp(name = "With_LED", group = "A")
public class WithF_NormalMode extends OpMode {

    // Hardware
    public DcMotorEx lf, rf, lb, rb, shooter;
    public CRServo leftAdvancer, rightAdvancer;
    public Servo prismLED;
    public VoltageSensor voltSensor;

    // Logic
    private boolean shooterOn = false;
    private boolean autoFaceOn = false;
    private double targetVelo = 0;

    private double pulseEndTime = 0;
    private double pulseDir = 0;
    private ElapsedTime pulseTimer = new ElapsedTime();

    private boolean lastRB2 = false, lastLB1 = false, lastX2 = false, lastB2 = false;

    // Constants
//    final double minD = 53, maxD = 121;
//    final double minV = 2400, maxV = 2780;

      final double minD = 53, maxD = 114;
  final double minV = 1600, maxV = 2025;
          ;
    private webCamOp camera;

    // LED PWM positions

//    private static final double ENDGAME_WARNING = 0.295;
//    private static final double ENDGAME_GOLD = 0.30;
    private static final double AUTO_FACE_SEARCH = 0.89;//blue locked
    private static final double AUTO_FACE_LOCKED = 0.87;//blue
    private static final double OUT_OF_RANGE = 0.61;// red
    private static final double SHOOTER_READY = 0.455;// green
    private static final double DEFAULT_ORANGE = 0.65;// orange

    // LED Modes (ONE winner only)
    enum LEDMode {
        ENDGAME_WARNING,
        ENDGAME_GOLD,
        AUTO_FACE_SEARCH,
        AUTO_FACE_LOCKED,
        AUTO_FACE_SHOOTER_READY,
        OUT_OF_RANGE,
        SHOOTER_READY,
        DEFAULT
    }

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

        lf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10)
        );

        camera = new webCamOp(hardwareMap);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        resetRuntime();
    }

    @Override
    public void loop() {
        camera.update();
        AprilTagDetection tag = getTargetTag();

        handleDrivetrain(tag);
        handleShooter(tag);
        handleAdvancers();
        updateLEDs(tag);

        telemetry.addData("Shooter On", shooterOn);
        telemetry.addData("Target Velo", targetVelo);
        telemetry.addData("Actual Velo", shooter.getVelocity());
        telemetry.addData("Auto Face", autoFaceOn);
        telemetry.update();
    }

    /* ---------------- DRIVETRAIN ---------------- */

    void handleDrivetrain(AprilTagDetection tag) {
        if (gamepad1.left_bumper && !lastLB1)
            autoFaceOn = !autoFaceOn;
        lastLB1 = gamepad1.left_bumper;

        double turn = gamepad1.right_stick_x;
        if (autoFaceOn && tag != null)
            turn = Range.clip(-tag.ftcPose.bearing * 0.02, -0.4, 0.4);

        double f = gamepad1.left_stick_y;
        double s = -gamepad1.left_stick_x;
        double d = Math.max(Math.abs(f) + Math.abs(s) + Math.abs(turn), 1);

        lf.setPower((f + s + turn) / d);
        rf.setPower((f - s - turn) / d);
        lb.setPower((f - s + turn) / d);
        rb.setPower((f + s - turn) / d);
    }

    /* ---------------- SHOOTER ---------------- */

    void handleShooter(AprilTagDetection tag) {
        if (gamepad2.right_bumper && !lastRB2)
            shooterOn = !shooterOn;
        lastRB2 = gamepad2.right_bumper;

        if (tag != null) {
            targetVelo = Range.clip(
                    ((tag.ftcPose.range - minD) / (maxD - minD)) * (maxV - minV) + minV,
                    minV, maxV
            );
        } else {
            targetVelo = maxV;
        }

        shooter.setVelocity(shooterOn ? targetVelo : 0);
    }

    /* ---------------- ADVANCERS ---------------- */

    void handleAdvancers() {
        if (shooter.getVelocity() > targetVelo-200) {
            if (gamepad2.x && !lastX2) {
                pulseDir = 1.0;
                pulseEndTime = pulseTimer.milliseconds() + 50;
            } else if (gamepad2.b && !lastB2) {
                pulseDir = -1.0;
                pulseEndTime = pulseTimer.milliseconds() + 50;
            }
        }


        lastX2 = gamepad2.x;
        lastB2 = gamepad2.b;

        if (pulseTimer.milliseconds() < pulseEndTime) {
            leftAdvancer.setPower(pulseDir);
            rightAdvancer.setPower(pulseDir);
        } else {
            leftAdvancer.setPower(0);
            rightAdvancer.setPower(0);
        }
    }

    /* ---------------- LED SYSTEM ---------------- */

    void updateLEDs(AprilTagDetection tag) {
        //double time = getRuntime();
        LEDMode mode = LEDMode.DEFAULT;

        boolean shooterReady =
                shooterOn && Math.abs(shooter.getVelocity() - targetVelo) < 60;

//        if (time > 115) {
//            mode = LEDMode.ENDGAME_WARNING;
//        }
//        else if (time > 90) {
//            mode = LEDMode.ENDGAME_GOLD;
//        }
         if (autoFaceOn) {
            if (tag == null)
                mode = LEDMode.AUTO_FACE_SEARCH;
            else if (shooterReady)
                mode = LEDMode.AUTO_FACE_SHOOTER_READY;
            else
                mode = LEDMode.AUTO_FACE_LOCKED;
        } else if (tag != null &&
                (tag.ftcPose.range < minD || tag.ftcPose.range > maxD)) {
            mode = LEDMode.OUT_OF_RANGE;
        } else if (shooterReady) {
            mode = LEDMode.SHOOTER_READY;
        }

        applyLED(mode);
    }

    void applyLED(LEDMode mode) {
        switch (mode) {
//            case ENDGAME_WARNING:
//                prismLED.setPosition(ENDGAME_WARNING);
//                break;
//            case ENDGAME_GOLD:
//                prismLED.setPosition(ENDGAME_GOLD);
//                break;
            case AUTO_FACE_SEARCH:
                prismLED.setPosition(AUTO_FACE_SEARCH);
                break;
            case AUTO_FACE_LOCKED:
                prismLED.setPosition(AUTO_FACE_LOCKED);
                break;
            case AUTO_FACE_SHOOTER_READY:
            case SHOOTER_READY:
                prismLED.setPosition(SHOOTER_READY);
                break;
            case OUT_OF_RANGE:
                prismLED.setPosition(OUT_OF_RANGE);
                break;
            default:
                prismLED.setPosition(DEFAULT_ORANGE);
        }
    }

    /* ---------------- TAG PICKER ---------------- */

    private AprilTagDetection getTargetTag() {
        if (camera.seesTag(20)) return camera.getTag(20);
        if (camera.seesTag(24)) return camera.getTag(24);
        return null;
    }
}
