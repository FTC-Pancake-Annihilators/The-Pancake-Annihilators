package org.firstinspires.ftc.teamcode.MainOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="GoBilda Mecanum TeleOp", group="Linear OpMode")
public class TeleOp_Mecanum extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftFrontMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightBackMotor = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note the strings used here must correspond
        // to the names assigned during the robot configuration step.
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back_motor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_motor");

        // Most motors consistently run counterclockwise when assigned a positive power value.
        // The right motors must be reversed to drive straight and strafe correctly (forming an "X" pattern).
        leftFrontMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Optional: Set motor modes (RUN_WITHOUT_ENCODER is common for simple TeleOp)
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the opmode
        while (opModeIsActive()) {

            // Get gamepad inputs
            double forward = -gamepad1.left_stick_y; // Y stick is usually reversed
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // Calculate motor powers using the mecanum equations
            double leftFrontPower = forward + strafe + turn;
            double rightFrontPower = forward - strafe - turn;
            double leftBackPower = forward - strafe + turn;
            double rightBackPower = forward + strafe - turn;

            // Normalize motor powers to ensure no value exceeds the -1 to 1 range
            double denominator = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower),
                    Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));
            if (denominator > 1.0) {
                leftFrontPower /= denominator;
                rightFrontPower /= denominator;
                leftBackPower /= denominator;
                rightBackPower /= denominator;
            }

            // Optional: Reduce speed by half if the right bumper is held
            if (gamepad1.right_bumper) {
                leftFrontPower /= 2.0;
                rightFrontPower /= 2.0;
                leftBackPower /= 2.0;
                rightBackPower /= 2.0;
            }

            // Set power to motors
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);

            // Telemetry for driver feedback (optional)
            telemetry.addData("Status", "Running");
            telemetry.addData("Left Front Power", leftFrontPower);
            telemetry.addData("Right Front Power", rightFrontPower);
            telemetry.addData("Left Back Power", leftBackPower);
            telemetry.addData("Right Back Power", rightBackPower);
            telemetry.update();
        }
    }
}




