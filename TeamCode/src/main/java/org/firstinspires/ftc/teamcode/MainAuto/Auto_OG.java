package org.firstinspires.ftc.teamcode.MainAuto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mecanum_Config;

@Autonomous(name="StarterBotAuto_Fixed", group="StarterBot")
public class Auto_OG extends OpMode {
    final double FEED_TIME = 0.20;
    final double LAUNCHER_TARGET_VELOCITY = 1750;
    final double LAUNCHER_MIN_VELOCITY = 1750;
    final double TIME_BETWEEN_SHOTS = 2;

    final double DRIVE_SPEED = 0.75;
    final double ROTATE_SPEED = 0.3;
    final double WHEEL_DIAMETER_MM = 104;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    int shotsToFire = 3;
    double robotRotationAngle = 45;

    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime feederTimer = new ElapsedTime();
    private ElapsedTime driveTimer = new ElapsedTime();



    private enum LaunchState { IDLE, PREPARE, LAUNCH }
    private LaunchState launchState;

    private enum AutonomousState {
        BACK_UP_BEFORE_SHOOT, // NEW: single back-up
        LAUNCH,
        WAIT_FOR_LAUNCH,
        DRIVING_AWAY_FROM_GOAL,
        ROTATING,
        DRIVING_OFF_LINE,
        COMPLETE
    }
    private AutonomousState autonomousState;

    private enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED;
    private Mecanum_Config mecanum;

    @Override
    public void init() {
       mecanum = new Mecanum_Config(hardwareMap);

        autonomousState = AutonomousState.BACK_UP_BEFORE_SHOOT;
        launchState = LaunchState.IDLE;

        mecanum.lf_Drive.setDirection(DcMotorEx.Direction.REVERSE);
        mecanum.rf_Drive.setDirection(DcMotorEx.Direction.FORWARD);
        mecanum.lb_Drive.setDirection(DcMotorEx.Direction.REVERSE);
        mecanum.rb_Drive.setDirection(DcMotorEx.Direction.FORWARD);

        mecanum.lf_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.rf_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.lb_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.rb_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        mecanum.lf_Drive.setZeroPowerBehavior(BRAKE);
        mecanum.rf_Drive.setZeroPowerBehavior(BRAKE);
        mecanum.lb_Drive.setZeroPowerBehavior(BRAKE);
        mecanum.rb_Drive.setZeroPowerBehavior(BRAKE);
        mecanum.shooter.setZeroPowerBehavior(BRAKE);

        mecanum.shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanum.lf_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanum.rf_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanum.lb_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mecanum.rb_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        mecanum.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        mecanum.leftAdvancer.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
        mecanum.leftAdvancer.setPower(0);
        mecanum.rightAdvancer.setPower(0);

        if (gamepad1.b) alliance = Alliance.RED;
        else if (gamepad1.x) alliance = Alliance.BLUE;

        telemetry.addData("Press X", "for BLUE");
        telemetry.addData("Press B", "for RED");
        telemetry.addData("Selected Alliance", alliance);
    }

    @Override
    public void start() { }

    @Override
    public void loop() {
        switch (autonomousState) {

            // -------- NEW: Back-up only once --------
            case BACK_UP_BEFORE_SHOOT:
                int ticksPerInch = 30;
                int backUpTicks = (int)-37 * ticksPerInch; // adjust for your robot
                mecanum.lf_Drive.setTargetPosition(backUpTicks);
                mecanum.rf_Drive.setTargetPosition(backUpTicks);
                mecanum.lb_Drive.setTargetPosition(backUpTicks);
                mecanum.rb_Drive.setTargetPosition(backUpTicks);

                mecanum.lf_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                mecanum.rf_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                mecanum.lb_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                mecanum.rb_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                mecanum.lf_Drive.setPower(0.6);
                mecanum.rf_Drive.setPower(0.6);
                mecanum.lb_Drive.setPower(0.6);
                mecanum.rb_Drive.setPower(0.6);

                // Move to next state once target reached
                if (Math.abs(mecanum.lf_Drive.getCurrentPosition() - backUpTicks) < 10) {
                    autonomousState = AutonomousState.LAUNCH;
                }
                break;

            case LAUNCH:
                launch(true);
                autonomousState = AutonomousState.WAIT_FOR_LAUNCH;
                break;

            case WAIT_FOR_LAUNCH:
                if (launch(false)) {
                    shotsToFire -= 1;
                    if (shotsToFire > 0) {
                        autonomousState = AutonomousState.LAUNCH;
                    } else {
                        mecanum.lf_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        mecanum.rf_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        mecanum.lb_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        mecanum.rb_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                        mecanum.shooter.setVelocity(0);
                        autonomousState = AutonomousState.DRIVING_AWAY_FROM_GOAL;
                    }
                }
                break;

            case DRIVING_AWAY_FROM_GOAL:
                if (drive(DRIVE_SPEED, -4, DistanceUnit.INCH, 1)) {
                    mecanum.lf_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    mecanum.rf_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    mecanum.lb_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    mecanum.rb_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.ROTATING;
                }
                break;

            case ROTATING:
                robotRotationAngle = (alliance == Alliance.RED) ? 45 : -45;
                if (rotate(ROTATE_SPEED, robotRotationAngle, AngleUnit.DEGREES, 1)) {
                    mecanum.lf_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    mecanum.rf_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    mecanum.lb_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    mecanum.rb_Drive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
                    autonomousState = AutonomousState.DRIVING_OFF_LINE;
                }
                break;

            case DRIVING_OFF_LINE:
                if (drive(DRIVE_SPEED, -26, DistanceUnit.INCH, 1)) {
                    autonomousState = AutonomousState.COMPLETE;
                }
                break;

            case COMPLETE:
                // do nothing
                break;
        }

        telemetry.addData("AutoState", autonomousState);
        telemetry.addData("LauncherState", launchState);
        telemetry.update();
    }

    @Override
    public void stop() { }

    boolean launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.PREPARE;
                    shotTimer.reset();
                }
                break;
            case PREPARE:
                mecanum.shooter.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (mecanum.shooter.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                    mecanum.leftAdvancer.setPower(1);
                    mecanum.rightAdvancer.setPower(1);
                    feederTimer.reset();
                }
                break;
            case LAUNCH:
                if (feederTimer.seconds() > FEED_TIME) {
                    mecanum.leftAdvancer.setPower(0);
                    mecanum.rightAdvancer.setPower(0);
                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
                        launchState = LaunchState.IDLE;
                        return true;
                    }
                }
                break;
        }
        return false;
    }

    boolean drive(double speed, double distance, DistanceUnit distanceUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetPosition = distanceUnit.toMm(distance) * TICKS_PER_MM;

        mecanum.lf_Drive.setTargetPosition((int) targetPosition);
        mecanum.rf_Drive.setTargetPosition((int) targetPosition);
        mecanum.lb_Drive.setTargetPosition((int) targetPosition);
        mecanum.rb_Drive.setTargetPosition((int) targetPosition);

        mecanum.lf_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        mecanum.rf_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        mecanum.lb_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        mecanum.rb_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        mecanum.lf_Drive.setPower(speed);
        mecanum.rf_Drive.setPower(speed);
        mecanum.lb_Drive.setPower(speed);
        mecanum.rb_Drive.setPower(speed);

        if (Math.abs(targetPosition - mecanum.lf_Drive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) driveTimer.reset();
        if (Math.abs(targetPosition - mecanum.rf_Drive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) driveTimer.reset();

        return (driveTimer.seconds() > holdSeconds);
    }

    boolean rotate(double speed, double angle, AngleUnit angleUnit, double holdSeconds) {
        final double TOLERANCE_MM = 10;
        double targetMm = angleUnit.toRadians(angle) * (TRACK_WIDTH_MM / 2);
        double leftTarget = -targetMm * TICKS_PER_MM;
        double rightTarget = targetMm * TICKS_PER_MM;

        mecanum.lf_Drive.setTargetPosition((int) leftTarget);
        mecanum.rf_Drive.setTargetPosition((int) rightTarget);
        mecanum.lb_Drive.setTargetPosition((int) leftTarget);
        mecanum.rb_Drive.setTargetPosition((int) rightTarget);

        mecanum.lf_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        mecanum.rf_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        mecanum.lb_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        mecanum.rb_Drive.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        mecanum.lf_Drive.setPower(speed);
        mecanum.rf_Drive.setPower(speed);
        mecanum.lb_Drive.setPower(speed);
        mecanum.rb_Drive.setPower(speed);

        if (Math.abs(leftTarget - mecanum.lf_Drive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) driveTimer.reset();
        if (Math.abs(leftTarget - mecanum.lb_Drive.getCurrentPosition()) > (TOLERANCE_MM * TICKS_PER_MM)) driveTimer.reset();

        return (driveTimer.seconds() > holdSeconds);
    }
}
