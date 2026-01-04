package org.firstinspires.ftc.teamcode.MainOp;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.ShooterState;
import org.firstinspires.ftc.teamcode.Mechanisms.subsystems;

import org.firstinspires.ftc.teamcode.util.AllianceColor;

@TeleOp(name = "Blue Alliance TeleOp", group = "Competition")
public class PPBlue extends LinearOpMode {

    subsystems mech;

    @Override
    public void runOpMode() {
        AllianceColor alliance = new AllianceColor(AllianceColor.Selection.BLUE);
        mech = new subsystems(hardwareMap, alliance);

        telemetry.addData("Status", "Initialized - Blue Alliance");
        telemetry.addData("Tip", "Press Play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            mech.drivetrain(gamepad1);
            mech.intake(gamepad2);
            mech.transfer(gamepad2);
            mech.shooter(gamepad2);
            mech.pathsAndModes(gamepad1);
            mech.cycle(gamepad2);

            Pose pose = mech.getFollower().getPose();

            telemetry.addLine("=== ROBOT POSE ===");
            telemetry.addData("X (inches)", "%.1f", pose.getX());
            telemetry.addData("Y (inches)", "%.1f", pose.getY());
            telemetry.addData("Heading (°)", "%.1f", Math.toDegrees(pose.getHeading()));

            telemetry.addLine("\n=== SHOOTER STATUS ===");
            telemetry.addData("Shooter State", subsystems.shooterState);
            if (subsystems.shooterState == ShooterState.AUTO_AIM) {
                telemetry.addData("Velocity", "Auto-adjusting to distance");
            }
            telemetry.addData("Defense Mode", mech.defenseActive ? "ON (Sideways lock)" : "OFF");

            telemetry.addLine("\n=== AUTO CYCLE ===");
            telemetry.addData("Cycle State", mech.cycleState);
            telemetry.addData("Shots Completed", mech.artifactsShot + "/3");
            if (mech.cycleState != subsystems.CycleState.IDLE) {
                telemetry.addData("Cycle Timer (s)", "%.2f", mech.cycleTimer.seconds());
            }

            telemetry.addLine("\n=== ACTIVE MODES ===");
            telemetry.addData("Auto Aim (Hold LB)", mech.autoAimActive ? "ON" : "OFF");

            String activePath = "None";
            if (mech.goToNearActive) activePath = "Near Zone";
            else if (mech.goToFarActive) activePath = "Far Zone";
            else if (mech.goToGateActive) activePath = "Gate (Ram)";
            else if (mech.parkActive) activePath = "Park";
            telemetry.addData("Active Path", activePath);
            telemetry.addData("Following Path", mech.getFollower().isBusy() ? "YES" : "NO");

            telemetry.addLine("\n=== MECHANISM STATUS ===");
            String intakeStatus = "OFF";
            if (gamepad2.a) intakeStatus = "FWD (Intaking)";
            else if (gamepad2.y) intakeStatus = "BWD (Ejecting)";
            telemetry.addData("Intake", intakeStatus);

            String transferStatus = "OFF";
            if (gamepad2.x) transferStatus = "FWD (Feed Out)";
            else if (gamepad2.b) transferStatus = "BWD (Reload In)";
            telemetry.addData("Transfer", transferStatus);

            telemetry.addLine("\n=== PATH BUTTONS (Gamepad 1) ===");
            telemetry.addData("Dpad Up", gamepad1.dpad_up ? "► Near Zone" : " ");
            telemetry.addData("Dpad Down", gamepad1.dpad_down ? "► Far Zone" : " ");
            telemetry.addData("Dpad Left", gamepad1.dpad_left ? "► Gate Ram" : " ");
            telemetry.addData("Dpad Right", gamepad1.dpad_right ? "► Park" : " ");
            telemetry.addData("B Button", gamepad1.b ? "► Pose Reset (83,88 @ 90°)" : " ");

            telemetry.addLine("\n=== QUICK TIPS ===");
            telemetry.addData("Right Bumper (gp2)", "Shooter ON + Defense");
            telemetry.addData("Right Trigger (gp2)", "Start 3-shot Cycle");

            telemetry.update();
        }
    }
}