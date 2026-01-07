package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.follower;

import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterState;
import org.firstinspires.ftc.teamcode.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

import java.util.function.Supplier;

public class subsystems {
    public Intake intake;
    public Shooter shooter;
    public Transfer transfer;

    public AllianceColor allianceColor;

    public boolean autoAimActive = false;
    public boolean goToNearActive = false;
    public boolean goToFarActive = false;
    public boolean goToGateActive = false;
    public boolean parkActive = false;
    public boolean defenseActive = false;

    public enum CycleState { IDLE, FEED, WAIT_SETTLE, WAIT_SHOOT }
    public CycleState cycleState = CycleState.IDLE;
    public int artifactsShot = 0;
    public ElapsedTime cycleTimer = new ElapsedTime();

    public static ShooterState shooterState = ShooterState.AUTO_AIM;

    private static PIDFController headingPid;

    // Made public for Auto OpModes
    public Supplier<PathChain> nearZonePath, farZonePath, parkPath, gatePath;

    private Servo prismLED;

    // PWM positions (test & adjust on robot from manual table)
    private static final double DEFAULT_ORANGE = 0.55;     // Solid orange idle
    private static final double CYCLE_FLASH = 0.295;       // Emergency Lights flashing
    private static final double DEFENSE_STROBE = 0.60;     // Purple/violet
    private static final double SHOOTER_READY = 0.70;      // Green pulse/sine wave
    private static final double PATH_RAINBOW = 1.0;        // Rainbow Snakes chase

    // Button tracking for toggles
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;
    private boolean lastRightTrigger = false;

    public subsystems(HardwareMap hwMap, AllianceColor color) {
        this.allianceColor = color;

        intake = new Intake(hwMap);
        shooter = new Shooter(hwMap, Constants.shooterCoefficients);
        transfer = new Transfer(hwMap);

        prismLED = hwMap.get(Servo.class, "prismLED");

        if (headingPid == null) {
            headingPid = new PIDFController(Constants.followerConstants.getCoefficientsHeadingPIDF());
        }

        definePaths();

        // Send PWM signal early to avoid white boot / snake demo
        prismLED.setPosition(DEFAULT_ORANGE);
    }

    public void updateLEDs() {
        if (cycleState != CycleState.IDLE) {
            prismLED.setPosition(CYCLE_FLASH);
            return;
        }
        if (defenseActive) {
            prismLED.setPosition(DEFENSE_STROBE);
            return;
        }
        if (shooterState == ShooterState.AUTO_AIM && autoAimActive) {
            prismLED.setPosition(SHOOTER_READY);
            return;
        }
        if (getFollower().isBusy()) {
            prismLED.setPosition(PATH_RAINBOW);
            return;
        }
        prismLED.setPosition(DEFAULT_ORANGE);  // Solid orange idle
    }

    public com.pedropathing.follower.Follower getFollower() {
        return follower;
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

    public void drivetrain(Gamepad gp1) {
        follower.update();

        double forward = -gp1.left_stick_y;
        double strafe = -gp1.left_stick_x;
        double turn = -gp1.right_stick_x;

        double stickMagnitude = Math.hypot(strafe, forward) + Math.abs(turn);

        // Cancel path if joystick >15%
        if (stickMagnitude > 0.15 && (goToNearActive || goToFarActive || goToGateActive || parkActive)) {
            goToNearActive = goToFarActive = goToGateActive = parkActive = false;
            follower.breakFollowing();
            follower.startTeleopDrive(true);
        }

        // Normal drive
        if (!autoAimActive && !goToNearActive && !goToFarActive && !goToGateActive && !parkActive && !defenseActive) {
            follower.setTeleOpDrive(forward, strafe, turn, false, allianceColor.isRed() ? 0 : Math.toRadians(180));
        }

        // Auto-face goal (toggle)
        if (gp1.left_bumper && !lastLeftBumper) {
            autoAimActive = !autoAimActive;
        }
        lastLeftBumper = gp1.left_bumper;

        if (autoAimActive) {
            headingPid.updatePosition(follower.getHeading());
            headingPid.setTargetPosition(getTargetHeading(follower.getPose().getX(), follower.getPose().getY(), allianceColor));
            follower.setTeleOpDrive(forward, strafe, headingPid.run(), false, allianceColor.isRed() ? 0 : Math.toRadians(180));
        }

        updateLEDs();
    }

    public void intake(Gamepad gp2) {
        if (gp2.a) {
            intake.intake();
        } else if (gp2.y) {
            intake.eject();
        } else {
            intake.stop();
        }
    }

    public void transfer(Gamepad gp2) {
        transfer.handleTimedAdvancers();
    }

    public void shooter(Gamepad gp2) {
        // Toggle defense + shooter
        if (gp2.right_bumper && !lastRightBumper) {
            defenseActive = !defenseActive;
            shooterState = ShooterState.AUTO_AIM;
        }
        lastRightBumper = gp2.right_bumper;

        if (defenseActive) {
            shooter.adaptive(follower.getPose().getX(), follower.getPose().getY(), allianceColor);
            headingPid.updatePosition(follower.getHeading());
            headingPid.setTargetPosition(allianceColor.isRed() ? Math.toRadians(90) : Math.toRadians(-90));
        }

        if (shooterState == ShooterState.AUTO_AIM) {
            shooter.adaptive(follower.getPose().getX(), follower.getPose().getY(), allianceColor);
        }

        updateLEDs();
    }

    public void pathsAndModes(Gamepad gp1) {
        // Paths (press to start)
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

        // Pose reset (alliance-specific)
        if (gp1.b) {
            if (allianceColor.isRed()) {
                follower.setPose(new Pose(8, 8, Math.toRadians(90)));
            } else {
                follower.setPose(new Pose(135, 8, Math.toRadians(90)));
            }
        }

        // Auto-complete
        if ((goToNearActive || goToFarActive || goToGateActive || parkActive) && !follower.isBusy()) {
            goToNearActive = goToFarActive = goToGateActive = parkActive = false;
        }

        updateLEDs();
    }

    private void cancelPathModes() {
        goToNearActive = goToFarActive = goToGateActive = parkActive = false;
    }

    public void cycle(Gamepad gp2) {
        // Toggle cycle
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

    private static double getTargetHeading(double x, double y, AllianceColor alliance) {
        if (alliance.isRed()) {
            return MathFunctions.normalizeAngle(Math.atan2(144 - y, 144 - x) + Math.toRadians(91.5));
        } else {
            return MathFunctions.normalizeAngle(Math.atan2(144 - y, 0 - x) + Math.toRadians(90));
        }
    }
}