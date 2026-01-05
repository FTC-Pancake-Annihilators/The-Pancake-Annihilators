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

    // Public for telemetry
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

    // Prism LED Driver - PWM mode (configured as Servo)
    private Servo prismLED;

    // PWM positions (servo.setPosition) based on manual table page 5
    // µsec to position: value / 2000 (e.g., 1000µsec = 0.5)
    // Tested starting points - fine-tune on your robot!
    private static final double ALLIANCE_RED = 0.55;        // Solid red (~1100µsec in solid color range)
    private static final double ALLIANCE_BLUE = 0.95;       // Solid blue (~1900µsec in solid color range)
    private static final double CYCLE_FLASH = 0.295;        // Emergency Lights flashing (580-589µsec ≈ 0.29-0.294)
    private static final double DEFENSE_STROBE = 0.60;      // Purple/violet solid (~1200µsec)
    private static final double SHOOTER_READY = 0.70;       // Sine Wave green pulse (~1400µsec)
    private static final double PATH_RAINBOW = 1.0;         // Rainbow Snakes chase (high end 2350-2500µsec, capped at 1.0)

    public subsystems(HardwareMap hwMap, AllianceColor color) {
        this.allianceColor = color;

        intake = new Intake(hwMap);
        shooter = new Shooter(hwMap, Constants.shooterCoefficients);
        transfer = new Transfer(hwMap);

        // Prism LED PWM mode - name "prismLED" in config
        prismLED = hwMap.get(Servo.class, "prismLED");

        if (headingPid == null) {
            headingPid = new PIDFController(Constants.followerConstants.getCoefficientsHeadingPIDF());
        }

        definePaths();

        setAllianceLED();  // Start with alliance color
    }

    private void setAllianceLED() {
        prismLED.setPosition(allianceColor.isRed() ? ALLIANCE_RED : ALLIANCE_BLUE);
    }

    // Update LEDs based on robot state - call every loop or after state changes
    public void updateLEDs() {
        if (cycleState != CycleState.IDLE) {
            prismLED.setPosition(CYCLE_FLASH);  // Flashing during auto cycle
            return;
        }

        if (defenseActive) {
            prismLED.setPosition(DEFENSE_STROBE);  // Purple for defense
            return;
        }

        if (shooterState == ShooterState.AUTO_AIM && autoAimActive) {
            prismLED.setPosition(SHOOTER_READY);  // Green pulse when shooter ready + aiming
            return;
        }

        if (getFollower().isBusy()) {
            prismLED.setPosition(PATH_RAINBOW);  // Rainbow chase while following path
            return;
        }

        setAllianceLED();  // Default = alliance color
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

        updateLEDs();
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

        updateLEDs();
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

        updateLEDs();
    }

    private static double getTargetHeading(double x, double y, AllianceColor alliance) {
        if (alliance.isRed()) {
            return MathFunctions.normalizeAngle(Math.atan2(141 - y, 141 - x) + Math.toRadians(91.5));
        } else {
            return MathFunctions.normalizeAngle(Math.atan2(141 - y, 3 - x) + Math.toRadians(90));
        }
    }
}