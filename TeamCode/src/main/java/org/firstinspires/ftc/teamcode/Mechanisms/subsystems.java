package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.util.function.Supplier;

public class subsystems {

    public Shooter shooter;
    public Transfer transfer;

    public AllianceColor allianceColor;

    private DcMotorEx leftFront, leftRear, rightFront, rightRear;

    public boolean goToNearActive = false;
    public boolean goToFarActive = false;
    public boolean goToGateActive = false;
    public boolean parkActive = false;

    public enum CycleState { IDLE, FEED, WAIT_SETTLE, WAIT_SHOOT }
    public CycleState cycleState = CycleState.IDLE;
    public int artifactsShot = 0;
    public ElapsedTime cycleTimer = new ElapsedTime();

    public Supplier<PathChain> nearZonePath, farZonePath, parkPath, gatePath;

    private Servo prismLED;

    private static final double DEFAULT_ORANGE = 0.55;
    private static final double CYCLE_FLASH = 0.295;
    private static final double DEFENSE_STROBE = 0.60;
    private static final double SHOOTER_READY = 0.70;
    private static final double PATH_RAINBOW = 1.0;

    private boolean lastRightTrigger = false;

    private static final double HEADING_GAIN_AIM = 3.2;
    private static final double HEADING_GAIN_STIFF = 6.0;
    private static final double AIM_TOLERANCE = Math.toRadians(6);

    private double targetHeading = 0;

    private static final double GOAL_X_RED = 140.0;
    private static final double GOAL_X_BLUE = 4.0;
    private static final double GOAL_Y = 140.0;

    public boolean shooterOn = false;
    public boolean aimingLocked = false;

    public subsystems() {
        // Empty constructor for OpMode framework
    }

    public subsystems(HardwareMap hwMap, AllianceColor color) {
        this.allianceColor = color;

        shooter = new Shooter(hwMap, Constants.shooterCoefficients);
        transfer = new Transfer(hwMap);

        leftFront = hwMap.get(DcMotorEx.class, "lf_Drive");
        leftRear = hwMap.get(DcMotorEx.class, "lb_Drive");
        rightFront = hwMap.get(DcMotorEx.class, "rf_Drive");
        rightRear = hwMap.get(DcMotorEx.class, "rb_Drive");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        prismLED = hwMap.get(Servo.class, "prismLED");

        definePaths();

        prismLED.setPosition(DEFAULT_ORANGE);
    }

    public void updateLEDs() {
        if (cycleState != CycleState.IDLE) {
            prismLED.setPosition(CYCLE_FLASH);
        } else if (shooterOn && aimingLocked) {
            prismLED.setPosition(SHOOTER_READY);
        } else if (follower.isBusy()) {
            prismLED.setPosition(PATH_RAINBOW);
        } else {
            prismLED.setPosition(DEFAULT_ORANGE);
        }
    }

    private void definePaths() {
        nearZonePath = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, allianceColor.isRed() ? Constants.redCloseShoot : Constants.blueCloseShoot)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,
                        allianceColor.isRed() ? Constants.redCloseShoot.getHeading() : Constants.blueCloseShoot.getHeading(), 0.8))
                .build();

        farZonePath = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, allianceColor.isRed() ? Constants.farRedShoot : Constants.farBlueShoot)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,
                        allianceColor.isRed() ? Constants.farRedShoot.getHeading() : Constants.farBlueShoot.getHeading(), 0.8))
                .build();

        parkPath = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, allianceColor.isRed() ? Constants.redPark : Constants.bluePark)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,
                        allianceColor.isRed() ? Constants.redPark.getHeading() : Constants.bluePark.getHeading(), 0.8))
                .build();

        gatePath = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose,
                        allianceColor.isRed() ? Constants.redGateApproach : Constants.blueGateApproach)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,
                        allianceColor.isRed() ? Constants.redGateApproach.getHeading() : Constants.blueGateApproach.getHeading(), 0.8))
                .build();
    }

    public void drivetrain(Gamepad gp1, Gamepad gp2) {
        follower.update();

        double forward = -gp1.left_stick_y;
        double strafe = gp1.left_stick_x;
        double turnInput = gp1.right_stick_x;

        double stickMagnitude = Math.hypot(forward, strafe) + Math.abs(turnInput);

        if (stickMagnitude > 0.15 && (goToNearActive || goToFarActive || goToGateActive || parkActive)) {
            goToNearActive = goToFarActive = goToGateActive = parkActive = false;
            follower.breakFollowing();
        }

        shooterOn = gp2.right_trigger > 0.1;
        boolean manualDefense = gp1.right_bumper;

        Pose currentPose = follower.getPose();
        double botHeading = currentPose.getHeading();

        double turnPower = turnInput;
        aimingLocked = false;

        if (shooterOn) {
            double desiredHeading = getTargetHeading(currentPose.getX(), currentPose.getY());
            double error = AngleWrap(desiredHeading - botHeading);

            double gain = Math.abs(error) < AIM_TOLERANCE ? HEADING_GAIN_STIFF : HEADING_GAIN_AIM;
            turnPower = gain * error;

            aimingLocked = Math.abs(error) < AIM_TOLERANCE;
        } else if (manualDefense) {
            double error = AngleWrap(targetHeading - botHeading);
            turnPower = HEADING_GAIN_STIFF * error;
        } else {
            turnPower = turnInput;
            targetHeading = botHeading;
        }

        if (!follower.isBusy()) {
            leftFront.setPower(forward + strafe + turnPower);
            leftRear.setPower(forward - strafe + turnPower);
            rightFront.setPower(forward - strafe - turnPower);
            rightRear.setPower(forward + strafe - turnPower);
        }

        updateLEDs();
    }

    public void shooterControl(Gamepad gp2) {
        if (shooterOn) {
            shooter.adaptive(follower.getPose().getX(), follower.getPose().getY(), allianceColor);
        } else {
            shooter.stop();
        }
    }

    public void pathsAndModes(Gamepad gp1) {
        if (gp1.dpad_up && !goToNearActive) {
            goToNearActive = true;
            cancelPathModes();
            follower.breakFollowing();
            follower.followPath(nearZonePath.get());
        }
        if (gp1.dpad_down && !goToFarActive) {
            goToFarActive = true;
            cancelPathModes();
            follower.breakFollowing();
            follower.followPath(farZonePath.get());
        }
        if (gp1.dpad_left && !goToGateActive) {
            goToGateActive = true;
            cancelPathModes();
            follower.breakFollowing();
            follower.followPath(gatePath.get());
        }
        if (gp1.dpad_right && !parkActive) {
            parkActive = true;
            cancelPathModes();
            follower.breakFollowing();
            follower.followPath(parkPath.get());
        }

        if (gp1.b) {
            follower.setPose(allianceColor.isRed()
                    ? new Pose(8, 8, Math.toRadians(90))
                    : new Pose(135, 8, Math.toRadians(90)));
        }

        if ((goToNearActive || goToFarActive || goToGateActive || parkActive) && !follower.isBusy()) {
            goToNearActive = goToFarActive = goToGateActive = parkActive = false;
        }

        updateLEDs();
    }

    private void cancelPathModes() {
        goToNearActive = goToFarActive = goToGateActive = parkActive = false;
    }

    public void cycle(Gamepad gp2) {
        if (gp2.right_trigger > 0.5 && !lastRightTrigger) {
            if (cycleState == CycleState.IDLE) {
                cycleState = CycleState.FEED;
                artifactsShot = 0;
                cycleTimer.reset();
            } else {
                cycleState = CycleState.IDLE;
                transfer.reload();
            }
        }
        lastRightTrigger = gp2.right_trigger > 0.5;

        switch (cycleState) {
            case IDLE:
                transfer.reload();
                break;
            case FEED:
                transfer.feed();
                if (cycleTimer.seconds() > 0.6) {
                    cycleState = CycleState.WAIT_SETTLE;
                    cycleTimer.reset();
                }
                break;
            case WAIT_SETTLE:
                transfer.feed();
                if (cycleTimer.seconds() > 0.3) {
                    cycleState = CycleState.WAIT_SHOOT;
                    cycleTimer.reset();
                }
                break;
            case WAIT_SHOOT:
                transfer.reload();
                if (cycleTimer.seconds() > 1.0) {
                    artifactsShot++;
                    if (artifactsShot >= 3) {
                        cycleState = CycleState.IDLE;
                    } else {
                        cycleState = CycleState.FEED;
                        cycleTimer.reset();
                    }
                }
                break;
        }

        updateLEDs();
    }

    private double getTargetHeading(double robotX, double robotY) {
        double goalX = allianceColor.isRed() ? GOAL_X_RED : GOAL_X_BLUE;
        double deltaX = goalX - robotX;
        double deltaY = GOAL_Y - robotY;
        return MathFunctions.normalizeAngle(Math.atan2(deltaY, deltaX));
    }

    private double AngleWrap(double angle) {
        return MathFunctions.normalizeAngle(angle);
    }

    public com.pedropathing.follower.Follower getFollower() {
        return follower;
    }
}