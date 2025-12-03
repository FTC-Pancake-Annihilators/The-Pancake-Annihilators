package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Mecanum_Config {
    public DcMotorEx lf_Drive;
    public DcMotorEx lb_Drive;
    public DcMotorEx rf_Drive;
    public DcMotorEx rb_Drive;
    public IMU imu;
    public DcMotor IntakeMotor;

    public DcMotorEx shooter;
    public CRServo leftAdvancer;
    public CRServo rightAdvancer;
    public double shooter_Velo = 1650;
    public double minimumVelo = shooter_Velo-50;
    public final double advancerPwr= 1.0;

    public boolean intakeOnfwd = false;
    public boolean intakeOnbwd = false;
    public boolean advancersOnfwd = false;
    public boolean advancersOnbwd = false;
    public boolean shooterOnfwd = false;
    public boolean shooterOnbwd = false;

    public String shooterStatus = "OFF";
    public String intakeStatus = "OFF";
    public String advancerStatus = "OFF";

    // Shooter Velo



    public static Pose startingPose;
    public Mecanum_Config(HardwareMap mecanum) {
        lf_Drive = mecanum.get(DcMotorEx.class, "lf_Drive");
        rf_Drive = mecanum.get(DcMotorEx.class, "rf_Drive");
        lb_Drive = mecanum.get(DcMotorEx.class, "lb_Drive");
        rb_Drive = mecanum.get(DcMotorEx.class, "rb_Drive");
        IntakeMotor = mecanum.get(DcMotor.class, "intake_motor");
        shooter = mecanum.get(DcMotorEx.class, "shooter");
        leftAdvancer = mecanum.get(CRServo.class, "leftAdvancer");
        rightAdvancer = mecanum.get(CRServo.class, "rightAdvancer");
        imu = mecanum.get(IMU.class, "imu");
        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu.initialize((new IMU.Parameters(revOrientation)));
        lf_Drive.setDirection(DcMotorEx.Direction.REVERSE);
        lb_Drive.setDirection(DcMotorEx.Direction.REVERSE);
        rf_Drive.setDirection(DcMotorEx.Direction.FORWARD);
        rb_Drive.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        lf_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lb_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lf_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rb_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rf_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));




    }


}
