package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Bot_2_Servo_LED_Logic")
public class With_Servo_LED extends OpMode {
    public DcMotorEx lf_Drive, lb_Drive, rf_Drive, rb_Drive;
    public DcMotorEx shooter;
    public CRServo leftAdvancer, rightAdvancer;
    public Servo blinkin;

    private boolean shooterMoving = false;
    private boolean autoFaceActive = false;
    private ElapsedTime pulseTimer = new ElapsedTime();
    private double pulseEndTime = 0;
    private double pulseDirection = 0;

    private boolean lastRightBumper2 = false, lastLeftBumper1 = false;
    private boolean lastX2 = false, lastB2 = false;
    private double targetShooterVelocity = 0;

    // LED PWM Values
    final double LED_RED = 0.6145;      // Shooter Off
    final double LED_GREEN = 0.7745;    // Shooter Ready Only
    final double LED_BLUE = 0.8745;     // Auto-Face Ready Only
    final double LED_GRN_PULSE = -0.07; // Ready to Fire (Both Ready)

    final double minDis = 53, maxDis = 121, minVelo = 2400, maxVelo = 2780;
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
        blinkin = hardwareMap.get(Servo.class, "blinkin");

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
    }

    public void updateLEDs() {
        // Check if shooter is on and within speed range
        boolean shooterReady = shooterMoving && (Math.abs(shooter.getVelocity() - targetShooterVelocity) < 60);
        // Check if Auto-Face is toggled and a tag is actually visible
        boolean faceReady = autoFaceActive && (getTargetTag() != null);

        if (shooterReady && faceReady) {
            blinkin.setPosition(0.4745); // BEATS_PER_MINUTE_FOREST_PALETTE
        } else if (shooterReady) {
            blinkin.setPosition(0.7745); // GREEN
        } else if (faceReady) {
            blinkin.setPosition(0.8745); // BLUE
        } else {
            blinkin.setPosition(0.6145); // RED
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