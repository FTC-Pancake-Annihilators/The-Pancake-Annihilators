package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp", group="TELEOP")
public class First_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        CRServo  rightAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");
        CRServo  leftAdvancer =  hardwareMap.get(CRServo.class, "leftAdvancer");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");

        double drive;
        double turn;
        double leftPower;
        double rightPower;
        boolean intakeOn;
        final double farShooter = 1; //place holder
        final double nearShooter = .5; //place holder






        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeOn = false;
        waitForStart();

        while (opModeIsActive()) {
            drive = gamepad1.left_stick_y * -1;
            turn = gamepad1.right_stick_x;
            rightPower = Range.clip(drive + turn, -1, 1);
            leftPower = Range.clip(drive - turn, -1,  1);

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);



            //intake

            if (gamepad2.xWasPressed() && !intakeOn) {
                intakeOn = true;


            }
            if (gamepad2.xWasPressed() && intakeOn) {
                intakeOn = false;
            }
            if (intakeOn) {
                intakeMotor.setPower(1);
                leftAdvancer.setPower(-1);
                rightAdvancer.setPower(1);

            } else {
                intakeMotor.setPower(0);
                leftAdvancer.setPower(0);
                rightAdvancer.setPower(0);
            }


            //Shooter
            if (gamepad2.right_bumper){
                shooter.setPower(farShooter);

            }
            else if (gamepad2.left_bumper){
                shooter.setPower(nearShooter);
            }
            else {
                shooter.setPower(0);
            }
        }
    }

}
