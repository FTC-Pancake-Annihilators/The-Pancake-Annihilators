package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mecanum_Config;

public class Mecanum_Drive extends OpMode {
    Mecanum_Config Mecanum_Config;

    @Override
    public void init() {
        Mecanum_Config = new Mecanum_Config(hardwareMap);
    }

    @Override
    public void loop() {

    }
}
