package org.firstinspires.ftc.teamcode.Mechanisms;



import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GoalAutoHeading{
    public Pose position;
    public Follower follower;
    private PIDFController headingPid;
    public void orbit(double posMultiplier, Pose goal, Gamepad gamepad) {
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



