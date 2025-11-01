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



package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp_Final_NoRumble", group="TELEOP")
public class TeleOp_Final extends LinearOpMode {

    private DcMotor leftMotor, rightMotor;
    private DcMotor intakeMotor;
    private CRServo leftAdvancer, rightAdvancer;
    private DcMotor shooter;

    private final double FAR_POWER  = 0.75;
    private final double NEAR_POWER = 0.4;

    // Status variables declared once at class level
    private String shooterStatus = "OFF";
    private String intakeStatus = "OFF";
    private String advancerStatus = "OFF";

    ///  Use Enums the way u are told.
    @Override
    public void runOpMode() {

        // DRIVE MOTORS
        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        shooter = hardwareMap.get(DcMotor.class, "shooter");

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // INTAKE + ADVANCERS
        intakeMotor   = hardwareMap.get(DcMotor.class, "intakeMotor");
        leftAdvancer  = hardwareMap.get(CRServo.class, "leftAdvancer");
        rightAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");
        rightAdvancer.setDirection(CRServo.Direction.REVERSE);

        // SHOOTER


        telemetry.addLine("Initialized ✅");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // --- DRIVE (Gamepad1) ---
            double drive =  gamepad1.left_stick_y;
            double turn  = gamepad1.right_stick_x;

            // Determine speed mode
            double speedScale = 0.7; // default full speed
            if (gamepad1.left_trigger > 0.2) speedScale = 0.5;  // slow
            if (gamepad1.right_trigger > 0.2) speedScale = 1.0; // normal (override slow)

            double leftPower  = Range.clip((drive*speedScale) - (turn*speedScale), -1, 1);
            double rightPower = Range.clip((drive*speedScale) + (turn*speedScale), -1, 1);

            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            // --- INTAKE (Gamepad2) ---
            if (gamepad2.y) {
                intakeMotor.setPower(1); // intake in
            } else if (gamepad2.a) {
                intakeMotor.setPower(-1); // intake out
            } else {
                intakeMotor.setPower(0);  // stop
            }

            // --- ADVANCERS (Gamepad2) ---
            if (gamepad2.x) {
                leftAdvancer.setPower(-1);
                rightAdvancer.setPower(1);
            } else {
                leftAdvancer.setPower(0);
                rightAdvancer.setPower(0);
            }

            // --- SHOOTER (Gamepad2) ---
            if (gamepad2.right_bumper) {
                shooter.setPower(FAR_POWER);
            } else if (gamepad2.left_bumper) {
                shooter.setPower(NEAR_POWER);
            } else {
                shooter.setPower(0);
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
