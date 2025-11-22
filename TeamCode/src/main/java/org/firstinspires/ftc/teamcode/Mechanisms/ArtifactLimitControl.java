package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Artifact Limit Control")
public class ArtifactLimitControl extends OpMode {

    DcMotorEx intake;
    DcMotorEx shooter;

    ColorRangeSensor intakeColor;
    ColorRangeSensor exitColor;

    int greenCount = 0;
    int purpleCount = 0;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        intakeColor = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        exitColor   = hardwareMap.get(RevColorSensorV3.class, "exitColor");

        telemetry.addLine("System Ready: 1 Green max, 2 Purple max");
    }

    @Override
    public void loop() {

        // ------------ INTAKE LOGIC ---------------

        String intakeDetected = detectColor(intakeColor);

        // Add GREEN if allowed
        if (intakeDetected.equals("GREEN") && greenCount < 1) {
            greenCount++;
        }

        // Add PURPLE if allowed
        if (intakeDetected.equals("PURPLE") && purpleCount < 2) {
            purpleCount++;
        }

        // ------------ INTAKE BLOCK RULES ----------
        boolean allowIntake = true;

        if (greenCount >= 1 && detectColor(intakeColor).equals("GREEN")) {
            allowIntake = false;
        }
        if (purpleCount >= 2 && detectColor(intakeColor).equals("PURPLE")) {
            allowIntake = false;
        }

        // If blocked, stop intake
        if (!allowIntake) {
            intake.setPower(0);
        } else {
            intake.setPower(gamepad1.right_trigger);
        }

        // ------------ SHOOTER RESET LOGIC ---------
        String exitDetected = detectColor(exitColor);

        if (exitDetected.equals("GREEN") && greenCount > 0) {
            greenCount--;
        }

        if (exitDetected.equals("PURPLE") && purpleCount > 0) {
            purpleCount--;
        }

        // ------------ TELEMETRY -------------------
        telemetry.addData("Green Count", greenCount);
        telemetry.addData("Purple Count", purpleCount);
        telemetry.addData("Intake Sees", intakeDetected);
        telemetry.addData("Exit Sees", exitDetected);
        telemetry.update();
    }


    // ---------------- COLOR DETECTION -----------------

    private String detectColor(ColorRangeSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        // Pure green (tune if needed)
        if (g > r + 20 && g > b + 20) return "GREEN";

        // Purple = Red + Blue mix
        if (b > g && r > g) return "PURPLE";

        return "NONE";
    }
}
