package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="Left Advancer Test")
public class leftAdvancerTest extends OpMode {
    private CRServo leftAdvancer;
    @Override
    public void init() {
        leftAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");
    }
    @Override
    public void loop() {
        if (gamepad1.a) {
            leftAdvancer.setPower(1);
        }
        else if (gamepad1.b){
            leftAdvancer.setPower(-1);
        }
        else {
            leftAdvancer.setPower(0);
        }
    }
}
