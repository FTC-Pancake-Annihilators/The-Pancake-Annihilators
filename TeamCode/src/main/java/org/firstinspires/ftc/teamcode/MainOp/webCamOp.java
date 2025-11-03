package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.Webcam_Localization;

public class webCamOp extends OpMode {

    Webcam_Localization aprilTagLocalization = new Webcam_Localization();

    @Override
    public void init() {
    aprilTagLocalization.initWebcamLocalization(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
    //Update the vision portal
    aprilTagLocalization.update();

    }
}
