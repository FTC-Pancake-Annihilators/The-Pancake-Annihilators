package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class distanceToSpeed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double minDis = 20;
        double maxDis = 180;
        double minVelo = 2000;
        double maxVelo = 6000;


        //slope intrecept
        double m =(maxVelo - minVelo) /(maxDis - minDis);
        double b = minVelo -m * minDis;

        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));


        waitForStart();

    while (opModeIsActive()){
        double distance = 100;//Placeholder for the getAprilTagDistance.

        double targetVelo = m * distance + b;

        targetVelo = Math.max(targetVelo,Math.min(maxVelo, targetVelo));

        shooter.setVelocity(targetVelo);

    }

    }
}
