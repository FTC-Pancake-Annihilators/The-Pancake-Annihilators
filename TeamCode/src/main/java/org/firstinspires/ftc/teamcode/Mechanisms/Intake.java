package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    public DcMotor intake;
    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Run the intake
     */
    public void intake() {
        intake.setPower(0.75);
    }

    /**
     * Eject an artifact in the robot
     */
    public void eject() {
        intake.setPower(-0.75);
    }

    public void stop() {
        intake.setPower(0);
    }
    public void custom(double speed) {
        intake.setPower(speed);
    }
}
