/// Gamepad1 – Driver (Drive):

   ///     Left stick Y → forward/backward

  ///      Right stick X → turn

   ///     Left trigger → slow mode (50%)

     ///   Right trigger → normal mode (100%, overrides slow)

     ///   Gamepad2 – Operator (Mechanisms):

    ///    Intake: Y = in, A = out, release = stop

     ///   Advancers: X = both in, release = stop

    ///    Shooter: RB = far (0.75), LB = near (0.4), release = stop

    ///    Double-blip rumble on shooter button press

   ///     Telemetry:

    ////    Combines Shooter / Intake / Advancers into one line

      ///  Shows Drive powers and Drive mode

       /// All motors stop immediately on release, as described.



package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Config;

@TeleOp(name="TeleOp_Final_NoRumble", group="TELEOP")
public class TeleOp_Final extends LinearOpMode {
    private Config config;


    private final double Far_POWER  = 0.75;
    private final double Far_Velo  = 1750;
    private final double Near_Velo  = Far_Velo / 2;

    private final double NEAR_POWER = 0.4;


    // Status variables declared once at class level
    private String shooterStatus = "OFF";
    private String intakeStatus = "OFF";
    private String advancerStatus = "OFF";

    ///  Use Enums the way u are told.
    @Override
    public void runOpMode() {
    config = new Config(hardwareMap);



        telemetry.addLine("Initialized ✅");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // --- DRIVE (Gamepad1) ---
            double drive =  gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;

            // Determine speed mode
            double speedScale = 0.7; // default full speed
            if (gamepad1.left_trigger > 0.2) speedScale = 0.4;  // slow
            if (gamepad1.right_trigger > 0.2) speedScale = 1.0; // normal (override slow)

            double leftPower  = Range.clip((drive*speedScale) - (turn*speedScale), -1, 1);
            double rightPower = Range.clip((drive*speedScale) + (turn*speedScale), -1, 1);

            config.leftMotor.setPower(leftPower);
            config.rightMotor.setPower(rightPower);

            // --- INTAKE (Gamepad2) ---
            if (gamepad2.y) {
                config.intakeMotor.setPower(1); // intake in
            } else if (gamepad2.a) {
                config.intakeMotor.setPower(-1); // intake out
            } else {
                config.intakeMotor.setPower(0);  // stop
            }

            // --- ADVANCERS (Gamepad2) ---
            if (gamepad2.x) {
                config.leftAdvancer.setPower(-1);
                config.rightAdvancer.setPower(1);
            } else {
                config.leftAdvancer.setPower(0);
                config.rightAdvancer.setPower(0);
            }
            if (gamepad2.b) {
                config.leftAdvancer.setPower(1);
                config.rightAdvancer.setPower(-1);
            } else {
                config.leftAdvancer.setPower(0);
                config.rightAdvancer.setPower(0);
            }

            // --- SHOOTER (Gamepad2) ---
            if (gamepad2.right_bumper) {
                config.shooter.setVelocity(Far_Velo);
            } else if (gamepad2.left_bumper) {
                config.shooter.setVelocity(Near_Velo);
            } else {
                config.shooter.setVelocity(0);
            }

            // --- TELEMETRY ---
            // Reassign values (no redeclaration)
            shooterStatus  = gamepad2.right_bumper ? "FAR" : gamepad2.left_bumper ? "NEAR" : "OFF";
            intakeStatus   = gamepad2.y ? "IN" : gamepad2.a ? "OUT" : "OFF";
            advancerStatus = gamepad2.x ? "IN" : "OFF";

            telemetry.addData("Status", "Shooter: %s | Intake: %s | Advancers: %s",
                    shooterStatus, intakeStatus, advancerStatus);
            telemetry.addData("Drive L/R", "L: %.2f R: %.2f", leftPower, rightPower);
            telemetry.addData("Drive Mode", gamepad1.left_trigger > 0.2 ? "SLOW (0.5)" : "NORMAL (1.0)");
            telemetry.update();
        }
    }
}
