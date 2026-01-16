package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.subsystems;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.util.AllianceColor.Selection;

@TeleOp(name = "PP-Red", group = "PP")
public class PPRed extends OpMode {

    private subsystems subs;

    @Override
    public void init() {
        subs = new subsystems(hardwareMap, new AllianceColor(Selection.RED));

        telemetry.addData("Alliance", "RED");
        telemetry.addData("Status", "Initialized – Hold RT to Auto Aim + Shoot!");
        telemetry.update();
    }

    @Override
    public void loop() {
        subs.drivetrain(gamepad1, gamepad2);
        subs.shooterControl(gamepad2);
        subs.pathsAndModes(gamepad1);
        subs.cycle(gamepad2);

        telemetry.addData("Pose", subs.getFollower().getPose().toString());
        telemetry.addData("Aiming Locked", subs.aimingLocked ? "YES (Stiff!)" : "Turning");
        telemetry.addData("Shooter On", subs.shooterOn ? "YES" : "Off");
        telemetry.addData("Cycle State", subs.cycleState);
        telemetry.update();
    }
}