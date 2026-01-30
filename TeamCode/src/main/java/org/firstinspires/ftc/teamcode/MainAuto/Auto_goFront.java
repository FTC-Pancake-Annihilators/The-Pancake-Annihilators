package org.firstinspires.ftc.teamcode.MainAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name ="GoFront",group = "AAA")
public class Auto_goFront extends OpMode {

    private DcMotorEx lf_Drive;
    private DcMotorEx rf_Drive;
    private DcMotorEx rb_Drive;
    private DcMotorEx lb_Drive;


    @Override
    public void init() {
        lf_Drive = hardwareMap.get(DcMotorEx.class, "lf_Drive");
        rf_Drive = hardwareMap.get(DcMotorEx.class, "rf_Drive");
        lb_Drive = hardwareMap.get(DcMotorEx.class, "lb_Drive");
        rb_Drive = hardwareMap.get(DcMotorEx.class, "rb_Drive");
        lf_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb_Drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb_Drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb_Drive.setDirection(DcMotor.Direction.REVERSE);
        rb_Drive.setDirection(DcMotor.Direction.FORWARD);
        lf_Drive.setDirection(DcMotor.Direction.REVERSE);
        rf_Drive.setDirection(DcMotor.Direction.FORWARD);


    }

    @Override
    public void loop() {

        lb_Drive.setTargetPosition(45 * 41);
        rb_Drive.setTargetPosition(45 * 41);
        lf_Drive.setTargetPosition(45 * 41);
        rf_Drive.setTargetPosition(45 * 41);

//        if (leftMotor.isBusy()) {
//            leftMotor.setPower(0.4);
//            rightMotor.setPower(0.4);
//        }

        lb_Drive.setPower(-0.4);
        rb_Drive.setPower(0.4);
        lf_Drive.setPower(-0.4);
        lf_Drive.setPower(0.4);


    }
}
