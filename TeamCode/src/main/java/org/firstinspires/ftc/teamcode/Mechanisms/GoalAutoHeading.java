package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.control.PIDFController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GoalAutoHeading{
    public Pose position;
    private PIDFController headingPid;
    private void orbit(double posMultiplier, Pose goal, Gamepad gamepad) {
        headingPid.updatePosition(position.getHeading());
        headingPid.setTargetPosition(calculateRobotCentricTargetHeading(goal));

        follower.setTeleOpDrive(
                -gamepad.left_stick_y * posMultiplier,
                gamepad.left_stick_x * posMultiplier,
                headingPid.run(),
                false               // Robot-centric
        );

    }
    public double calculateRobotCentricTargetHeading(Pose target) {
        double adjacent = Math.abs(target.getX() - position.getX());
        double opposite = Math.abs(target.getY() - position.getY());
        return opposite / adjacent;
    }
}



