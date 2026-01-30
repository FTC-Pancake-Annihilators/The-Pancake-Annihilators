//package org.firstinspires.ftc.teamcode.MainAuto;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Mecanum_Config;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//
//@Configurable
//public class IZ_Red_Artafacts_Auto extends OpMode {
//    private enum LaunchState { IDLE, PREPARE, LAUNCH }
//    private LaunchState launchState = LaunchState.IDLE;
//    private ElapsedTime shotTimer = new ElapsedTime();
//    private ElapsedTime feederTimer = new ElapsedTime();
//
//    private static final double LAUNCHER_TARGET_VELOCITY = 1750;
//    private static final double LAUNCHER_MIN_VELOCITY = 1749;
//    private static final double FEED_TIME = 0.2;
//    private static final double TIME_BETWEEN_SHOTS = 2;
//
//    private boolean shooting = false; // tracks when we are shooting
//    private int shotsToFire = 3; // tracks shots per shooting segment
//    private TelemetryManager panelsTelemetry;
//    public Follower follower;
//    private int pathState;
//    private Paths paths;
//    private Mecanum_Config mecanum;
//
//    @Override
//    public void init() {
//        mecanum = new Mecanum_Config(hardwareMap);
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));
//
//        paths = new Paths(follower);
//
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
//        mecanum.shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        pathState = autonomousPathUpdate();
//
//        // Telemetry
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
//        panelsTelemetry.debug("Shooting", shooting);
//        panelsTelemetry.debug("LaunchState", launchState);
//        panelsTelemetry.debug("Shots Left", shotsToFire);
//        panelsTelemetry.update(telemetry);
//    }
//
//    public int autonomousPathUpdate() {
//        switch (pathState) {
//
//            case 0: // Go to first goal
//                follower.followPath(paths.gotogoalshootdefault);
//                pathState++;
//                break;
//
//            case 1: // Shooting first set
//                if (!shooting) {
//                    shooting = true;
//                    shotsToFire = 3;
//                }
//                if (shooting) {
//                    if (launch(true)) {
//                        shotsToFire--;
//                        if (shotsToFire <= 0) {
//                            shooting = false;
//                            pathState++;
//                        }
//                    }
//                }
//                break;
//
//            case 2:
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.GotoSet1);
//                    pathState++;
//                }
//                break;
//
//            case 3: // Intake first artifacts
//                mecanum.IntakeMotor.setPower(1);
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Intakeset1);
//                    pathState++;
//                }
//                break;
//
//            case 4: // Move to shooting position set 1
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Gotoshootset1);
//                    shooting = true;
//                    shotsToFire = 3;
//                    pathState++;
//                }
//                break;
//
//            case 5: // Shooting set 1
//                if (shooting) {
//                    if (launch(true)) {
//                        shotsToFire--;
//                        if (shotsToFire <= 0) {
//                            shooting = false;
//                            pathState++;
//                        }
//                    }
//                }
//                break;
//
//            case 6: // Go to set 2
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.GotoSet2);
//                    pathState++;
//                }
//                break;
//
//            case 7: // Intake set 2
//                mecanum.IntakeMotor.setPower(1);
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Intakeset2);
//                    pathState++;
//                }
//                break;
//
//            case 8: // Move to shooting position set 2
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Gotoshootset2);
//                    shooting = true;
//                    shotsToFire = 3;
//                    pathState++;
//                }
//                break;
//
//            case 9: // Shooting set 2
//                if (shooting) {
//                    if (launch(true)) {
//                        shotsToFire--;
//                        if (shotsToFire <= 0) {
//                            shooting = false;
//                            pathState++;
//                        }
//                    }
//                }
//                break;
//
//            case 10: // Go to set 3
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.GotoSet3);
//                    pathState++;
//                }
//                break;
//
//            case 11: // Intake set 3
//                mecanum.IntakeMotor.setPower(1);
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Intakeset3);
//                    pathState++;
//                }
//                break;
//
//            case 12: // Move to shooting position set 3
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.Gotoshootset3);
//                    shooting = true;
//                    shotsToFire = 3;
//                    pathState++;
//                }
//                break;
//
//            case 13: // Shooting set 3
//                if (shooting) {
//                    if (launch(true)) {
//                        shotsToFire--;
//                        if (shotsToFire <= 0) {
//                            shooting = false;
//                            pathState++;
//                        }
//                    }
//                }
//                break;
//
//            case 14: // Move to ranking point
//                if (!follower.isBusy()) {
//                    follower.followPath(paths.GetRankingPoint);
//                    pathState++;
//                }
//                break;
//
//            case 15:
//                // All paths finished
//                break;
//        }
//
//        return pathState;
//    }
//    boolean launch(boolean shotRequested) {
//        switch (launchState) {
//            case IDLE:
//                if (shotRequested) {
//                    launchState = LaunchState.PREPARE;
//                    shotTimer.reset();
//                }
//                break;
//            case PREPARE:
//                mecanum.shooter.setVelocity(LAUNCHER_TARGET_VELOCITY);
//                if (mecanum.shooter.getVelocity() > LAUNCHER_MIN_VELOCITY) {
//                    launchState = LaunchState.LAUNCH;
//                    mecanum.leftAdvancer.setPower(1);
//                    mecanum.rightAdvancer.setPower(1);
//                    feederTimer.reset();
//                }
//                break;
//            case LAUNCH:
//                if (feederTimer.seconds() > FEED_TIME) {
//                    mecanum.leftAdvancer.setPower(0);
//                    mecanum.rightAdvancer.setPower(0);
//                    if (shotTimer.seconds() > TIME_BETWEEN_SHOTS) {
//                        launchState = LaunchState.IDLE;
//                        return true;
//                    }
//                }
//                break;
//        }
//        return false;
//    }
//    public static class Paths {
//
//        public PathChain gotogoalshootdefault;
//        public PathChain GotoSet1;
//        public PathChain Intakeset1;
//        public PathChain Gotoshootset1;
//        public PathChain GotoSet2;
//        public PathChain Intakeset2;
//        public PathChain Gotoshootset2;
//        public PathChain GotoSet3;
//        public PathChain Intakeset3;
//        public PathChain Gotoshootset3;
//        public PathChain GetRankingPoint;
//
//        public Paths(Follower follower) {
//            gotogoalshootdefault = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(80.180, 10.023),
//                                    new Pose(76.053, 77.675),
//                                    new Pose(96.540, 95.509)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
//                    .build();
//
//            GotoSet1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(96.540, 95.509),
//                                    new Pose(69.568, 33.752),
//                                    new Pose(100.815, 34.931)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//                    .build();
//
//            Intakeset1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(100.815, 34.931), new Pose(122.923, 34.342))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .build();
//
//            Gotoshootset1 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(122.923, 34.342),
//                                    new Pose(67.799, 25.056),
//                                    new Pose(96.540, 95.361)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//                    .build();
//
//            GotoSet2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(96.540, 95.361),
//                                    new Pose(73.842, 58.072),
//                                    new Pose(97.425, 59.251)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//                    .build();
//
//            Intakeset2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(97.425, 59.251), new Pose(118.796, 58.956))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .build();
//
//            Gotoshootset2 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(118.796, 58.956),
//                                    new Pose(100.373, 53.945),
//                                    new Pose(96.540, 95.214)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//                    .build();
//
//            GotoSet3 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(96.540, 95.214),
//                                    new Pose(87.107, 82.981),
//                                    new Pose(95.951, 84.749)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
//                    .build();
//
//            Intakeset3 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(95.951, 84.749), new Pose(120.418, 83.275))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
//                    .build();
//
//            Gotoshootset3 = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierLine(new Pose(120.418, 83.275), new Pose(96.688, 95.361))
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
//                    .build();
//
//            GetRankingPoint = follower
//                    .pathBuilder()
//                    .addPath(
//                            new BezierCurve(
//                                    new Pose(96.688, 95.361),
//                                    new Pose(108.921, 38.469),
//                                    new Pose(73.842, 44.659)
//                            )
//                    )
//                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(270))
//                    .build();
//        }
//    }
//}
