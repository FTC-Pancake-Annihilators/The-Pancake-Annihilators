package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


public class webCamOp {

    private AprilTagProcessor processer;

    private VisionPortal visionportal;

    private List<AprilTagDetection> detections;
    private AprilTagDetection[] detectionMap;

    public webCamOp(HardwareMap hardwareMap) {
        processer = new AprilTagProcessor.Builder().build();

        visionportal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(processer)
                .build();

    }


    public void update() {
        detections = processer.getDetections();
        detectionMap = new AprilTagDetection[5];
        for (AprilTagDetection detection : detections) {
            //if detection.id is a between 20 and 24 add to the map
            if (detection.id >19 && detection.id <25){
                detectionMap[detection.id - 20] = detection;
            }
        }
    }

    public boolean seesTag(int id) {
        return detectionMap[id-20] != null;
    }

    public AprilTagDetection getTag(int id) {
        return detectionMap[id-20];
    }

    public List<AprilTagDetection> getAllDetections() {
        return detections;
    }
}