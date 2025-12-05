package org.firstinspires.ftc.teamcode.MainAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "RED_Final_Auto_With_Parking", group = "Main")
public class Red_newbot_auto extends LinearOpMode {

    private DcMotorEx shooter;
    private Servo leftServo, rightServo;
    private Follower follower;
    private PathChain path1, path2; // ✅ fixed

    private VisionPortal portal;
    private AprilTagProcessor detector;
    private final int TAG_ID_RED = 10;

    private double baseRPM = 2500.0;
    private double multiplier = 900.0;
    private final double rpmTolerance = 5.0;

    private final double leftHome = 0.0;
    private final double rightHome = 1.0;
    private final double servoTap = 0.03;
    private final int tapHoldMs = 120;
    private final int betweenShotsMs = 220;

    private final double GOAL_OFFSET_METERS = 0.10;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware init
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        leftServo = hardwareMap.get(Servo.class, "leftservo");
        rightServo = hardwareMap.get(Servo.class, "rightservo");
        leftServo.setPosition(leftHome);
        rightServo.setPosition(rightHome);

        follower = Constants.createFollower(hardwareMap);

        // Path1 – Bezier to shooting
        path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(80.180, 8.843),
                                new Pose(77.085, 78.854),
                                new Pose(95.803, 96.098)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build(); // ✅ returns PathChain

        // Vision init
        detector = new AprilTagProcessor.Builder().build();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // ✅ fixed import
                .addProcessor(detector)
                .build();

        telemetry.addLine("Init complete");
        telemetry.update();

        waitForStart();

        // Drive Path1
        follower.followPath(path1);
        while (opModeIsActive() && follower.isBusy()) follower.update();

        // Scan AprilTag 10
        AprilTagDetection foundTag = null;
        while (opModeIsActive() && foundTag == null) {
            List<AprilTagDetection> detections = detector.getDetections();
            for (AprilTagDetection d : detections) if (d.id == TAG_ID_RED) { foundTag = d; break; }
            telemetry.addLine("Scanning for Tag 10...");
            telemetry.update();
        }

        if (!opModeIsActive() || foundTag == null) return;

        double measuredDistance = foundTag.ftcPose.range;
        double goalDistance = measuredDistance + GOAL_OFFSET_METERS;
        double targetRPM = baseRPM + (goalDistance * multiplier);

        setShooterRPM(targetRPM);

        sleep(300); // spin-up

        // 3-shot sequence
        for (int shot = 1; shot <= 3 && opModeIsActive(); shot++) {
            while (opModeIsActive() && Math.abs(getShooterRPM() - targetRPM) > rpmTolerance) {
                setShooterRPM(targetRPM);
                sleep(20);
            }

            leftServo.setPosition(clamp(leftHome + servoTap,0,1));
            rightServo.setPosition(clamp(rightHome - servoTap,0,1));
            sleep(tapHoldMs);

            leftServo.setPosition(leftHome);
            rightServo.setPosition(rightHome);

            sleep(betweenShotsMs);
        }

        // Wait 3s
        sleep(3000);

        // Path2 – parking
        path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(46.870, 96.540),
                                new Pose(32.721, 78.117)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build(); // ✅ returns PathChain

        follower.followPath(path2);
        while (opModeIsActive() && follower.isBusy()) follower.update();

        telemetry.addLine("Auto complete. Parked.");
        telemetry.update();
    }

    private void setShooterRPM(double rpm) {
        double ticksPerRev = shooter.getMotorType().getTicksPerRev();
        shooter.setVelocity((rpm / 60.0) * ticksPerRev);
    }

    private double getShooterRPM() {
        double ticksPerRev = shooter.getMotorType().getTicksPerRev();
        return (shooter.getVelocity() / ticksPerRev) * 60.0;
    }

    private double clamp(double val, double min, double max) { return Math.max(min, Math.min(max, val)); }
}
