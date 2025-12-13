package org.firstinspires.ftc.teamcode.MainOp;

import android.icu.text.Transliterator;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class VisionStuff extends OpMode {

    private webCamOp vision;
    @Override
    public void init() {
        vision = new webCamOp(hardwareMap);

    }

    @Override
    public void loop() {
        vision.update();
        if (vision.seesTag(21)){

        }

        if (vision.seesTag(20)){
            AprilTagDetection detection = vision.getTag(20);
            Position pos = detection.robotPose.getPosition();
            double distance = Math.sqrt(pos.x*pos.x+pos.y*pos.y);
        }
    }
}
