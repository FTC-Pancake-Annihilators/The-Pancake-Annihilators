package org.firstinspires.ftc.teamcode.MainAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mecanum_Config;

@Autonomous(name="Far_Blue_Auto_Align", group = "AAA")
public class Auto_Align_Far_Blue extends LinearOpMode {

    Mecanum_Config mecanum;

    @Override
    public void runOpMode() throws InterruptedException {

        mecanum = new Mecanum_Config(hardwareMap);

        waitForStart();

        if (!opModeIsActive()) return;
        drive(false,true,200);
        // Fire 3 shots
        fire();
        sleep(2500);
        fire();
        sleep(2500);
        fire();

        // Turn shooter off
        mecanum.shooter.setVelocity(0);

        // Strafe to color side
        drive(true, false, 3000);
    }

    /** FIRE ONE ARTF **/
    public void fire() {

        if (!opModeIsActive()) return;

        // Spin shooter up
        mecanum.shooter.setVelocity(2000);

        ElapsedTime timer = new ElapsedTime();


        while (opModeIsActive()
                && mecanum.shooter.getVelocity() < 1990
                && timer.seconds() < 2.5) {
            sleep(5);
        }

        if (!opModeIsActive()) return;

        // Feed ONE ARTF
        mecanum.leftAdvancer.setPower(-mecanum.advancerPwr);
        mecanum.rightAdvancer.setPower(-mecanum.advancerPwr);
        //mecanum.IntakeMotor.setPower(-1);

        sleep(635);  // enough for 1 ARTF

        // Stop feed Servos
        mecanum.leftAdvancer.setPower(0);
        mecanum.rightAdvancer.setPower(0);
        //mecanum.IntakeMotor.setPower(0);
    }

    /** DRIVE STRAIGHT OR TURN **/
    public void drive(boolean turn, boolean alignturn, long millis) {

        if (!opModeIsActive()) return;

        if (turn) {
            // Rotation
            mecanum.lb_Drive.setPower(-0.5);
            mecanum.rb_Drive.setPower(0.5);
            mecanum.lf_Drive.setPower(0.5);
            mecanum.rf_Drive.setPower(-0.5);
        } else if (alignturn) {
            mecanum.lb_Drive.setPower(0.2);
            mecanum.rb_Drive.setPower(0);
            mecanum.lf_Drive.setPower(0.2);
            mecanum.rf_Drive.setPower(0);
        } else {
            // no motion
            mecanum.lb_Drive.setPower(0);
            mecanum.rb_Drive.setPower(0);
            mecanum.lf_Drive.setPower(0);
            mecanum.rf_Drive.setPower(0);
        }

        sleep(millis);

        // Stop motors
        mecanum.lb_Drive.setPower(0);
        mecanum.rb_Drive.setPower(0);
        mecanum.lf_Drive.setPower(0);
        mecanum.rf_Drive.setPower(0);
    }
}
