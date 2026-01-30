package org.firstinspires.ftc.teamcode.MainAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mecanum_Config;

@Autonomous(name="Far_blue",group = "normal Auto")
public class BallTryblue extends LinearOpMode {

    Mecanum_Config mecanum;

    @Override
    public void runOpMode() throws InterruptedException {

        mecanum = new Mecanum_Config(hardwareMap);

        waitForStart();

        if (!opModeIsActive()) return;
        // drive straight
        drive(false,false,false,400);
        //turn to align the intake to intake.
        drive(false,true,false,300);
        //drive to intake and intake.
        mecanum.IntakeMotor.setPower(1);
        // Fire 3 shots
        fire();
        sleep(1500);
        fire();
        sleep(1500);
        fire();

        // Turn shooter off
        mecanum.shooter.setVelocity(0);

        // Drive backwards
        drive(true, false,false,3000);// change this later
    }


    public void fire() {

        if (!opModeIsActive()) return;

        // Spin shooter up
        mecanum.shooter.setVelocity(mecanum.shooter_Velo);

        ElapsedTime timer = new ElapsedTime();

        // Warm up shooter WITH SAFETY
        while (opModeIsActive()
                && mecanum.shooter.getVelocity() < mecanum.minimumVelo
                && timer.seconds() < 2.5) {
            sleep(5);
        }

        if (!opModeIsActive()) return;


        mecanum.leftAdvancer.setPower(-mecanum.advancerPwr);
        mecanum.rightAdvancer.setPower(-mecanum.advancerPwr);
        //mecanum.IntakeMotor.setPower(-1);

        sleep(800);  // enough for 1 ring

        // Stop feed motors
        mecanum.leftAdvancer.setPower(0);
        mecanum.rightAdvancer.setPower(0);
        //mecanum.IntakeMotor.setPower(0);
    }

    /** DRIVE STRAIGHT OR TURN **/
    public void drive(boolean turn,boolean reverseturn,boolean alignturn, long millis) {

        if (!opModeIsActive()) return;

        if (turn) {
            // Rotation
            mecanum.lb_Drive.setPower(-0.5);
            mecanum.rb_Drive.setPower(0.5);
            mecanum.lf_Drive.setPower(-0.5);
            mecanum.rf_Drive.setPower(0.5);

        }
        else if (reverseturn) {
            mecanum.lb_Drive.setPower(0.5);
            mecanum.rb_Drive.setPower(-0.5);
            mecanum.lf_Drive.setPower(0.5);
            mecanum.rf_Drive.setPower(-0.5);
        }

        else if (alignturn) {
            mecanum.lb_Drive.setPower(0.2);
            mecanum.rb_Drive.setPower(0);
            mecanum.lf_Drive.setPower(0.2);
            mecanum.rf_Drive.setPower(0);
        } else {
            // Straight motion
            mecanum.lb_Drive.setPower(-0.5);
            mecanum.rb_Drive.setPower(-0.5);
            mecanum.lf_Drive.setPower(-0.5);
            mecanum.rf_Drive.setPower(-0.5);
        }

        sleep(millis);

        // Stop motors
        mecanum.lb_Drive.setPower(0);
        mecanum.rb_Drive.setPower(0);
        mecanum.lf_Drive.setPower(0);
        mecanum.rf_Drive.setPower(0);
    }
}
