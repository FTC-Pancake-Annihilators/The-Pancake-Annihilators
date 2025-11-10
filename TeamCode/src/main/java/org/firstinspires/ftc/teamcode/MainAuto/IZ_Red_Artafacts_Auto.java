package org.firstinspires.ftc.teamcode.MainAuto;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Red Goal Auto", group = "Autonomous")
@Configurable
public class IZ_Red_Artafacts_Auto extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build red-side paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Run state machine

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.gotogoalshootdefault);
                pathState++;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GotoSet1);
                    pathState++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Intakeset1);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Gotoshootset1);
                    pathState++;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GotoSet2);
                    pathState++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Intakeset2);
                    pathState++;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Gotoshootset2);
                    pathState++;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GotoSet3);
                    pathState++;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Intakeset3);
                    pathState++;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Gotoshootset3);
                    pathState++;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.GetRankingPoint);
                    pathState++;
                }
                break;

            case 11:
                // All paths complete
                break;
        }
        return pathState;
    }

    public static class Paths {

        public PathChain gotogoalshootdefault;
        public PathChain GotoSet1;
        public PathChain Intakeset1;
        public PathChain Gotoshootset1;
        public PathChain GotoSet2;
        public PathChain Intakeset2;
        public PathChain Gotoshootset2;
        public PathChain GotoSet3;
        public PathChain Intakeset3;
        public PathChain Gotoshootset3;
        public PathChain GetRankingPoint;

        public Paths(Follower follower) {
            gotogoalshootdefault = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(80.180, 10.023),
                                    new Pose(76.053, 77.675),
                                    new Pose(96.540, 95.509)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                    .build();

            GotoSet1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(96.540, 95.509),
                                    new Pose(69.568, 33.752),
                                    new Pose(100.815, 34.931)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Intakeset1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(100.815, 34.931), new Pose(122.923, 34.342))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Gotoshootset1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(122.923, 34.342),
                                    new Pose(67.799, 25.056),
                                    new Pose(96.540, 95.361)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            GotoSet2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(96.540, 95.361),
                                    new Pose(73.842, 58.072),
                                    new Pose(97.425, 59.251)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Intakeset2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(97.425, 59.251), new Pose(118.796, 58.956))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Gotoshootset2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(118.796, 58.956),
                                    new Pose(100.373, 53.945),
                                    new Pose(96.540, 95.214)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            GotoSet3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(96.540, 95.214),
                                    new Pose(87.107, 82.981),
                                    new Pose(95.951, 84.749)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            Intakeset3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(95.951, 84.749), new Pose(120.418, 83.275))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Gotoshootset3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(120.418, 83.275), new Pose(96.688, 95.361))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            GetRankingPoint = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(96.688, 95.361),
                                    new Pose(108.921, 38.469),
                                    new Pose(73.842, 44.659)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(270))
                    .build();
        }
    }
}
