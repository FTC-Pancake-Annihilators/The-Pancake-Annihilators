package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mecanum_Config;

public class Field_Orientation {

    private Mecanum_Config mecanum;
    public void init(HardwareMap hardwareMap) {
        mecanum = new Mecanum_Config(hardwareMap);

    }
    public void drive(double forward, double strafe, double turn){
        double frontLeftPower = forward + strafe + turn;
        double backLeftPower = forward - strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backRightPower = forward + strafe - turn;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        mecanum.lf_Drive.setPower(maxSpeed * (frontLeftPower / maxPower));
        mecanum.lb_Drive.setPower(maxSpeed * (backLeftPower / maxPower));
        mecanum.rf_Drive.setPower(maxSpeed * (frontRightPower / maxPower));
        mecanum.rb_Drive.setPower(maxSpeed * (backRightPower / maxPower));

    }
    public void driveFieldRelative(double forward, double strafe, double turn){
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta -
                mecanum.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r* Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, turn);
    }
}

