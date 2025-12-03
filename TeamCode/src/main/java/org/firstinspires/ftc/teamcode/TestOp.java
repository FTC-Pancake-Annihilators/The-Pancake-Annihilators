package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TestOp extends OpMode {
    @Override
    public void init() {
        DcMotor leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        DcMotor rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        CRServo rightAdvancer = hardwareMap.get(CRServo.class,"rightAdvancer");
        CRServo leftAdvancer = hardwareMap.get(CRServo.class,"leftAdvancer");
        DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");

    }

    @Override
    public void loop() {

    }
}
