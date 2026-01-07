package org.firstinspires.ftc.teamcode.MainOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@TeleOp(name = "PP-Red")
public class PPRed extends LinearOpMode {

    subsystems mech;

    @Override
    public void runOpMode() {
        AllianceColor alliance = new AllianceColor(AllianceColor.Selection.RED);
        mech = new subsystems(hardwareMap, alliance);

        telemetry.addData("Status", "Initialized - Red Alliance");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            mech.drivetrain(gamepad1);
            mech.intake(gamepad2);
            mech.transfer(gamepad2);
            mech.shooter(gamepad2);
            mech.pathsAndModes(gamepad1);
            mech.cycle(gamepad2);

            mech.updateLEDs();

            Pose pose = mech.getFollower().getPose();

            telemetry.addLine("=== ROBOT POSE ===");
            telemetry.addData("X", "%.1f", pose.getX());
            telemetry.addData("Y", "%.1f", pose.getY());
            telemetry.addData("Heading °", "%.1f", Math.toDegrees(pose.getHeading()));

            telemetry.addLine("\n=== SHOOTER ===");
            telemetry.addData("State", subsystems.shooterState);
            telemetry.addData("Target Velocity", "%.0f", mech.shooter.getTargetVelocity());
            telemetry.addData("Defense", mech.defenseActive ? "ON" : "OFF");

            telemetry.addLine("\n=== CYCLE ===");
            telemetry.addData("State", mech.cycleState);
            telemetry.addData("Shots", mech.artifactsShot + "/3");
            if (mech.cycleState != subsystems.CycleState.IDLE) {
                telemetry.addData("Timer s", "%.2f", mech.cycleTimer.seconds());
            }

            telemetry.addLine("\n=== MODES ===");
            telemetry.addData("Auto Aim", mech.autoAimActive ? "ON" : "OFF");

            String path = "None";
            if (mech.goToNearActive) path = "Near";
            else if (mech.goToFarActive) path = "Far";
            else if (mech.goToGateActive) path = "Gate";
            else if (mech.parkActive) path = "Park";
            telemetry.addData("Path", path);
            telemetry.addData("Following", mech.getFollower().isBusy() ? "YES" : "NO");

            telemetry.addLine("\n=== MECHANISMS ===");
            String intakeStatus = gamepad2.a ? "FWD" : gamepad2.y ? "BWD" : "OFF";
            telemetry.addData("Intake", intakeStatus);

            String transferStatus = gamepad2.x ? "FWD (Feed)" : gamepad2.b ? "BWD (Reload)" : "OFF";
            telemetry.addData("Transfer", transferStatus);

            telemetry.addLine("\n=== BUTTONS ===");
            telemetry.addData("Dpad Up", gamepad1.dpad_up ? "► Near" : "");
            telemetry.addData("Dpad Down", gamepad1.dpad_down ? "► Far" : "");
            telemetry.addData("Dpad Left", gamepad1.dpad_left ? "► Gate" : "");
            telemetry.addData("Dpad Right", gamepad1.dpad_right ? "► Park" : "");
            telemetry.addData("B", gamepad1.b ? "► Pose Reset" : "");

            telemetry.update();
        }
    }
}