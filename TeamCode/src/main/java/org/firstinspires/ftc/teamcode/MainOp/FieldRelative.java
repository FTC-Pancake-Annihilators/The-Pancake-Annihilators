package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mecanum_Config;
import org.firstinspires.ftc.teamcode.Mechanisms.Mecanum_Drive;

public class FieldRelative extends OpMode {

     Field_Orientation drive = new Field_Orientation();
    private Mecanum_Config mecanum;

    double forward, strafe, turn;

    @Override
    public void init() {
        drive.init(hardwareMap);
        mecanum = new Mecanum_Config(hardwareMap);
    }

    @Override
    public void loop() {
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        turn = gamepad1.right_stick_x;

        drive.driveFieldRelative(forward,strafe,turn);




        if (gamepad2.y) {
            mecanum.IntakeMotor.setPower(1); // intake in
        } else if (gamepad2.a) {
            mecanum.IntakeMotor.setPower(-1); // intake out
        } else {
            mecanum.IntakeMotor.setPower(0);  // stop
        }

        // Advancers forward toggle (X)
        if (gamepad2.x) {
            mecanum.leftAdvancer.setPower(-1);
            mecanum.rightAdvancer.setPower(-1);
        } else {
            mecanum.leftAdvancer.setPower(0);
            mecanum.rightAdvancer.setPower(0);
        }
        if (gamepad2.b) {
            mecanum.leftAdvancer.setPower(1);
            mecanum.rightAdvancer.setPower(-1);
        } else {
            mecanum.leftAdvancer.setPower(0);
            mecanum.rightAdvancer.setPower(0);
        }

        // Shooter forward toggle (right bumper)
        if (gamepad2.rightBumperWasPressed()) {
            mecanum.shooterOnfwd = !mecanum.shooterOnfwd;
            if (mecanum.shooterOnfwd) mecanum.shooterOnbwd = false; // mutually exclusive
        }

        // Shooter reverse toggle (left bumper)
        if (gamepad2.leftBumperWasPressed()) {
            mecanum.shooterOnbwd = !mecanum.shooterOnbwd;
            if (mecanum.shooterOnbwd) mecanum.shooterOnfwd = false; // mutually exclusive
        }

        // ---------- Apply motor outputs (set each motor once) ----------
        // Intake motor
        double intakePower = 0.0;
        if (mecanum.intakeOnfwd) intakePower = 1.0;
        else if (mecanum.intakeOnbwd) intakePower = -1.0;
        mecanum.IntakeMotor.setPower(intakePower);

        // Advancers / feeders
        double leftAdvPower = 0.0;
        double rightAdvPower = 0.0;
        if (mecanum.advancersOnfwd) {
            leftAdvPower = -1.0;
            rightAdvPower = 1.0;
        } else if (mecanum.advancersOnbwd) {
            leftAdvPower = 1.0;
            rightAdvPower = -1.0;
        }
        mecanum.leftAdvancer.setPower(leftAdvPower);
        mecanum.rightAdvancer.setPower(rightAdvPower);

        // Shooter (use velocity setting once)
        double shooterVelocity = 0.0;
        if (mecanum.shooterOnfwd) shooterVelocity = mecanum.shooter_Velo;
        else if (mecanum.shooterOnbwd) shooterVelocity = -mecanum.shooter_Velo;
        // Assuming Mecanum_Config.shooter is a DcMotorEx with setVelocity method
        mecanum.shooter.setVelocity(shooterVelocity);

        // ---------- Status strings (reflect toggles, not instantaneous buttons) ----------
        mecanum.shooterStatus = mecanum.shooterOnfwd ? "FORWARD" : mecanum.shooterOnbwd ? "REVERSE" : "OFF";
        mecanum.intakeStatus = mecanum.intakeOnfwd ? "IN" : mecanum.intakeOnbwd ? "OUT" : "OFF";
        mecanum.advancerStatus = mecanum.advancersOnfwd ? "IN" : mecanum.advancersOnbwd ? "OUT" : "OFF";

        telemetry.addData("Status", "Shooter: %s | Intake: %s | Advancers: %s",
                mecanum.shooterStatus, mecanum.intakeStatus, mecanum.advancerStatus);
        telemetry.update();

        // ---------- Automated PathFollowing ----------

    }
}
