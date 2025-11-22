package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name="FastColorSortShooter")
public class Air_sorting extends LinearOpMode {

    private DcMotorEx shooter;
    private NormalizedColorSensor colorSensor;

    // CHANGE THIS for your motor’s ticks-per-rev
    final double TPR = 28.0;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        waitForStart();

        // Spin up to default RPM (helps reach color-specific RPM faster)
        double currentTargetRPM = 3500;
        shooter.setVelocity(rpmToTPS(currentTargetRPM));

        while (opModeIsActive()) {

            // WHEN YOU PRESS A BUTTON → shoot 3 balls quickly
            if (gamepad1.a) {
                shootThreeBalls();
            }

            telemetry.addData("RPM", getRPM());
            telemetry.update();
        }
    }

    // ================================
    // SHOOT 3 BALLS WITH 0.2 sec gap
    // ================================
    private void shootThreeBalls() {

        for (int i = 1; i <= 3; i++) {

            // 1) Read the color of THIS ball
            String color = detectColor(colorSensor.getNormalizedColors());

            // 2) Choose RPM based only on color
            double targetRPM = pickRPM(color);

            // 3) Adjust shooter speed (rapid switching)
            shooter.setVelocity(rpmToTPS(targetRPM));

            // 4) WAIT for RPM to be stable
            waitForRPMStable(targetRPM, 50); // 50 RPM tolerance

            // 5) FIRE THIS BALL
            fireBall();

            // 6) Delay only 200 ms before next
            sleep(200);
        }
    }

    // Fire function — modify for your feeder servo
    private void fireBall() {
        // Example for servo:
        // feed.setPosition(1);
        // sleep(80);
        // feed.setPosition(0);
    }

    // ================================
    // HELPER FUNCTIONS
    // ================================

    private String detectColor(NormalizedRGBA c) {
        if (c.red > c.blue + 0.04) return "RED";
        if (c.blue > c.red + 0.04) return "BLUE";
        return "UNKNOWN";
    }

    private double pickRPM(String color) {
        switch (color) {
            case "RED":  return 3400;
            case "BLUE": return 3600;
            default:     return 3500;
        }
    }

    // Convert RPM → ticks per second
    private double rpmToTPS(double rpm) {
        return (rpm * TPR) / 60.0;
    }

    private double getRPM() {
        return (shooter.getVelocity() * 60.0) / TPR;
    }

    // Wait until the shooter reaches the RPM zone
    private void waitForRPMStable(double target, double tolerance) {
        long timeout = System.currentTimeMillis() + 500; // 0.5 sec max
        while (opModeIsActive() && System.currentTimeMillis() < timeout) {
            double rpm = getRPM();
            if (Math.abs(rpm - target) <= tolerance) return;
        }
    }
}
