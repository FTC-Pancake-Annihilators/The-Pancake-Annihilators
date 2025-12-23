package org.firstinspires.ftc.teamcode.MainAuto;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Mecanum_Config;


public class Blue_Auto extends LinearOpMode {
    final double FEED_TIME = 0.20;
    final double LAUNCHER_TARGET_VELOCITY = 1755;
    final double LAUNCHER_MIN_VELOCITY = 1750;
    final double TIME_BETWEEN_SHOTS = 2;

    final double DRIVE_SPEED = 0.75;
    final double ROTATE_SPEED = 0.3;
    final double WHEEL_DIAMETER_MM = 104;
    final double ENCODER_TICKS_PER_REV = 537.7;
    final double TICKS_PER_MM = (ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI));
    final double TRACK_WIDTH_MM = 404;

    int shotsToFire = 3;
    double robotRotationAngle = 45;

    Mecanum_Config mecanum;
    @Override
    public void runOpMode() throws InterruptedException {
        opModeIsActive();
        mecanum = new Mecanum_Config(hardwareMap);

        waitForStart();
        //drive(false, 1500);
        fire();
        sleep(1000);
        fire();
        sleep(1000);
        fire();
        mecanum.shooter.setVelocity(0);
        drive(true, 500);
        drive(false, 1000);
    }

    public void fire() {
        mecanum.shooter.setVelocity(mecanum.shooter_Velo);
        while (mecanum.shooter.getVelocity() < mecanum.minimumVelo) sleep(3);
        mecanum.leftAdvancer.setPower(mecanum.advancerPwr);
        mecanum.rightAdvancer.setPower(-mecanum.advancerPwr);
        mecanum.IntakeMotor.setPower(-1);
        sleep(5000);
        mecanum.shooter.setPower(0);
        mecanum.rightAdvancer.setPower(0);
        mecanum.leftAdvancer.setPower(0);


    }
    public void drive(boolean turn, long millis) {
            if (turn){
                mecanum.lb_Drive.setPower(-0.5);
                mecanum.rb_Drive.setPower(-0.5);
                mecanum.lf_Drive.setPower(0.5);
                mecanum.rf_Drive.setPower(0.5);
            }
//        if (turn) {
//            mecanum.lb_Drive.setPower(0.5);
//            mecanum.rb_Drive.setPower(-0.5);
//            mecanum.lf_Drive.setPower(0.5);
//            mecanum.rf_Drive.setPower(-0.5);
//        } else {
//            mecanum.lb_Drive.setPower(-0.5);
//            mecanum.rb_Drive.setPower(-0.5);
//            mecanum.lf_Drive.setPower(-0.5);
//            mecanum.rf_Drive.setPower(-0.5);
//        }

        sleep(millis);
        mecanum.lb_Drive.setPower(0);
        mecanum.rb_Drive.setPower(0);
        mecanum.lf_Drive.setPower(0);
        mecanum.rf_Drive.setPower(0);

    }
}
