
//
//Gamepad 1Left Stick YDrivetrainForward
// / Backward Drive-gamepad1.left_stick_yGamepad
// 1Right Stick XDrivetrainTurning / Steering+gamepad1.right_stick_x-------
// --------Gamepad 2Right BumperIntake MotorIntake In (Collects game pieces)intakeMotor.setPower(1.0)
// Gamepad 2Left BumperIntake MotorOuttake / Reverse (Clears jams)intakeMotor.setPower(-1.0)
// Gamepad 2Y ButtonShooter MotorSpin Up ShootershooterMotor.setPower(SHOOTER_SPEED)
// Gamepad 2A ButtonFeeder CR ServosFeed Game Piece (To shooter)feederServo1/2.setPower(1.0)
// Gamepad 2B ButtonFeeder CR ServosFeeder Reverse (Clears jams)feederServo1/2.setPower(-1.0)



package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Dual Controller Robot TeleOp")
public class TeleOp extends LinearOpMode {

    // --- 1. DECLARE HARDWARE ---
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotor shooterMotor = null;
    private CRServo feederServo1 = null;
    private CRServo feederServo2 = null;

    // Define a constant for the shooter speed (Tune this value: 0.0 to 1.0)
    final double SHOOTER_SPEED = 0.8;

    @Override
    public void runOpMode() {

        // --- 2. INITIALIZE HARDWARE ---
        // Replace "name_in_config" with the actual name from your Control Hub
        leftDrive    = hardwareMap.get(DcMotor.class, "leftMotor");
        rightDrive   = hardwareMap.get(DcMotor.class, "rightMotor");
        intakeMotor  = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
        feederServo1 = hardwareMap.get(CRServo.class, "rightAdvancer");
        feederServo2 = hardwareMap.get(CRServo.class, "leftAdvancer");

        // --- 3. SET MOTOR DIRECTIONS ---
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        shooterMotor.setDirection(DcMotor.Direction.FORWARD);

        // Optional: Ensure drivetrains stop instantly for better control
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized. Driver: Gamepad 1, Mech: Gamepad 2");
        telemetry.update();

        waitForStart();

        // --- 4. THE MAIN CONTROL LOOP ---
        while (opModeIsActive()) {

            // =======================================================
            // A. GAMEPAD 1: DRIVE CONTROLS (Arcade Style)
            // =======================================================

            // Calculate raw joystick inputs
            double drive = -gamepad1.left_stick_y;  // Fwd/Bkd movement
            double turn  =  gamepad1.right_stick_x; // Left/Right turning

            // Calculate power for each motor
            double leftPower = drive + turn;
            double rightPower = drive - turn;

            // Clip power values to stay within the legal range of -1.0 to 1.0
            leftPower  = Range.clip(leftPower, -1.0, 1.0);
            rightPower = Range.clip(rightPower, -1.0, 1.0);

            // Send calculated power to the drivetrain motors
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);


            // =======================================================
            // B. GAMEPAD 2: MECHANISM CONTROLS
            // =======================================================

            // --- INTAKE CONTROL ---
            if (gamepad2.right_bumper) {
                intakeMotor.setPower(1.0);     // Intake In
            } else if (gamepad2.left_bumper) {
                intakeMotor.setPower(-1.0);    // Outtake/Reverse
            } else {
                intakeMotor.setPower(0.0);
            }

            // --- SHOOTER CONTROL ---
            // Y button spins up the shooter
            if (gamepad2.y) {
                shooterMotor.setPower(SHOOTER_SPEED);
            } else {
                shooterMotor.setPower(0.0);
            }

            // --- FEEDER (CR SERVO) CONTROL ---
            // A button runs the feeders to push game pieces into the shooter
            if (gamepad2.a) {
                feederServo1.setPower(1.0);
                feederServo2.setPower(1.0);
            }
            // B button reverses the feeder (useful for clearing jams)
            else if (gamepad2.b) {
                feederServo1.setPower(-1.0);
                feederServo2.setPower(-1.0);
            }
            else {
                feederServo1.setPower(0.0);
                feederServo2.setPower(0.0);
            }

            // --- TELEMETRY ---
            telemetry.addData("Drive Power (L/R)", "%.2f / %.2f", leftPower, rightPower);
            telemetry.addData("Intake Power", "%.1f", intakeMotor.getPower());
            telemetry.addData("Shooter Power", "%.1f (Y Button)", shooterMotor.getPower());
            telemetry.update();
        }
    }
}