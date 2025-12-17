package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.MainOp.webCamOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

@TeleOp(name = "Webcam Test")
public class distanceToSpeed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Distance and velocity ranges
        double minDis = 20;        // minimum distance (mm)
        double maxDis = 180;       // maximum distance (mm)
        double minVelo = 2000;     // min shooter velocity
        double maxVelo = 10000;    // max shooter velocity

        double distance = Double.NaN;  // Initialize distance safely

        // Initialize vision
        webCamOp vision = new webCamOp(hardwareMap);

        // Initialize shooter motor
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(300, 0, 0, 10));

        waitForStart();

        while (opModeIsActive()) {

            // Update vision
            vision.update();

            if (vision.seesTag(22)) {
                // Get AprilTag detection
                AprilTagDetection detection = vision.getTag(22);
                AprilTagPoseFtc pos = detection.ftcPose;

                // Compute planar distance
                distance = pos.range;

                if (distance >= minDis) {
                    // Compute target velocity based on linear scaling
                    double targetVelo = ((distance - minDis) / (maxDis - minDis)) *
                            (maxVelo - minVelo) + minVelo;

                    // Clamp velocity
                    targetVelo = Math.min(Math.max(targetVelo, minVelo), maxVelo);

                    // Set shooter velocity
                    shooter.setVelocity(targetVelo);
                } else {
                    shooter.setVelocity(minVelo); // Optional: minimum spin
                }

            } else {
                // No tag detected → stop shooter
                shooter.setVelocity(0);
                distance = Double.NaN;
            }

            // Telemetry
            telemetry.addData("Distance", Double.isNaN(distance) ? "No Tag" : distance);
            telemetry.update();
        }
    }
}
