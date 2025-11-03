package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="HeadingOpMode", group="TELEOP")
public class HeadingOpMode extends OpMode {
    boolean HeadingOn = false;
    @Override
    public void init() {

    }

    @Override
    public void loop() {

   if (gamepad1.aWasPressed() && HeadingOn){
            HeadingOn = true;
        }
        if (gamepad1.aWasPressed() && !HeadingOn){
            HeadingOn = false;
        }
    }
}
