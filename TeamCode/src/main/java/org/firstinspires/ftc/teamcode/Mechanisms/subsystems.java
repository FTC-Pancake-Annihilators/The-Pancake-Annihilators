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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;  // I²C driver
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.Artboard;  // Artboard enum

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

    private Supplier<PathChain> nearZonePath, farZonePath, parkPath, gatePath;

    // Prism RGB LED Driver (I²C mode - full dynamic patterns)
    private GoBildaPrismDriver prism;

    public subsystems(HardwareMap hwMap, AllianceColor color) {
        this.allianceColor = color;

        intake = new Intake(hwMap);
        shooter = new Shooter(hwMap, Constants.shooterCoefficients);
        transfer = new Transfer(hwMap);

        // Initialize Prism I²C driver (configure as "prism" in RC phone, type GoBildaPrismDriver)
        prism = hwMap.get(GoBildaPrismDriver.class, "prism");

        // Optional: Disable boot animation for full code control (set once)
        // prism.enableDefaultBootArtboard(false);

        if (headingPid == null) {
            headingPid = new PIDFController(Constants.followerConstants.getCoefficientsHeadingPIDF());
        }

        definePaths();

        // Initial LED: Alliance Artboard
        setAllianceArtboard();
    }

    // Set alliance-specific Artboard (pre-created with Prism Configurator)
    private void setAllianceArtboard() {
        if (allianceColor.isRed()) {
            prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_0);  // Red theme (solid, pulse, etc.)
        } else {
            prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_1);  // Blue theme
        }
    }

    // Dynamic LED update with priority (like your RevBlinkin example)
    public void updateLEDs() {
        // PRIORITY 1: Cycle active → Flash yellow/orange (Artboard 2)
        if (cycleState != CycleState.IDLE) {
            prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_2);
            return;
        }

        // PRIORITY 2: Defense mode → Purple strobe (Artboard 3)
        if (defenseActive) {
            prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_3);
            return;
        }

        // PRIORITY 3: Shooter ready + auto-aim → Green heartbeat (Artboard 4)
        if (shooterState == ShooterState.AUTO_AIM && autoAimActive) {
            prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_4);
            return;
        }

        // PRIORITY 4: Path following → Rainbow chase (Artboard 5)
        if (getFollower().isBusy()) {
            prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_5);
            return;
        }

        // DEFAULT: Alliance color
        setAllianceArtboard();
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
                        allianceColor.isRed()
                                ? new Pose(120, 30, Math.toRadians(180))
                                : new Pose(24, 114, Math.toRadians(0))
                )))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,
                        allianceColor.isRed() ? Math.toRadians(180) : Math.toRadians(0), 0.8))
                .build();
    }

    public void drivetrain(Gamepad gp1) {
        follower.update();

        double forward = -gp1.left_stick_y;
        double strafe = -gp1.left_stick_x;
        double turn = -gp1.right_stick_x;

        if (!autoAimActive && !goToNearActive && !goToFarActive && !goToGateActive && !parkActive && !defenseActive) {
            follower.setTeleOpDrive(forward, strafe, turn, false, allianceColor.isRed() ? 0 : Math.toRadians(180));
        }

        if (gp1.left_bumper) {
            autoAimActive = true;
            headingPid.updatePosition(follower.getHeading());
            headingPid.setTargetPosition(getTargetHeading(follower.getPose().getX(), follower.getPose().getY(), allianceColor));
            follower.setTeleOpDrive(forward, strafe, headingPid.run(), false, allianceColor.isRed() ? 0 : Math.toRadians(180));
        } else {
            autoAimActive = false;
        }

        updateLEDs();  // Update on auto-aim change
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
        if (gp2.right_bumper) {
            shooterState = ShooterState.AUTO_AIM;
            defenseActive = true;
            autoSpeed();

            headingPid.updatePosition(follower.getHeading());
            headingPid.setTargetPosition(allianceColor.isRed() ? Math.toRadians(90) : Math.toRadians(-90));
        } else {
            defenseActive = false;
        }

        if (shooterState == ShooterState.AUTO_AIM) {
            autoSpeed();
        }

        updateLEDs();  // Update on shooter/defense change
    }

    private void autoSpeed() {
        Pose pose = follower.getPose();
        shooter.adaptive(pose.getX(), pose.getY(), allianceColor);
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
            follower.setPose(new Pose(83, 88, Math.toRadians(90)));
        }

        if ((goToNearActive || goToFarActive || goToGateActive || parkActive) && !follower.isBusy()) {
            goToNearActive = goToFarActive = goToGateActive = parkActive = false;
        }

        updateLEDs();  // Update on path start/complete
    }

    private void cancelPathModes() {
        goToNearActive = goToFarActive = goToGateActive = parkActive = false;
    }

    public void cycle(Gamepad gp2) {
        if (gp2.right_trigger > 0.5 && cycleState == CycleState.IDLE) {
            cycleState = CycleState.FEED;
            artifactsShot = 0;
            cycleTimer.reset();
        }

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

        updateLEDs();  // Flash on cycle start/change
    }

    private static double getTargetHeading(double x, double y, AllianceColor alliance) {
        if (alliance.isRed()) {
            return MathFunctions.normalizeAngle(Math.atan2(141 - y, 141 - x) + Math.toRadians(91.5));
        } else {
            return MathFunctions.normalizeAngle(Math.atan2(141 - y, 3 - x) + Math.toRadians(90));
        }
    }
}