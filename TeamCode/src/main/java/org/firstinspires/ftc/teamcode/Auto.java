package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="My Robot Autonomous - Shoot First")
public class Auto extends LinearOpMode {

    // --- 1. HARDWARE DECLARATIONS (Same as before) ---
    private DcMotor leftDrive    = null;
    private DcMotor rightDrive   = null;
    private DcMotor intakeMotor  = null;
    private DcMotor shooterMotor = null;
    private CRServo feederServo1 = null;
    private CRServo feederServo2 = null;

    private ElapsedTime runtime = new ElapsedTime();

    // --- 2. ENCODER CONSTANTS (MUST BE CALCULATED) ---
    static final double COUNTS_PER_MOTOR_REV = 537.7;  // EXAMPLE VALUE
    static final double DRIVE_GEAR_REDUCTION = 1.0;    // EXAMPLE VALUE
    static final double WHEEL_DIAMETER_INCHES = 4.0;   // EXAMPLE VALUE
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    // --- 3. DRIVE AND MECHANISM SPEEDS ---
    static final double DRIVE_SPEED     = 0.6;
    static final double SHOOTER_SPEED   = 0.9;

    @Override
    public void runOpMode() {

        // --- 4. HARDWARE INITIALIZATION (Same as before) ---
        leftDrive    = hardwareMap.get(DcMotor.class, "leftMotor");
        rightDrive   = hardwareMap.get(DcMotor.class, "rightMotor");
        intakeMotor  = hardwareMap.get(DcMotor.class, "intakeMotor");
        shooterMotor = hardwareMap.get(DcMotor.class, "shooter");
        feederServo1 = hardwareMap.get(CRServo.class, "rightAdvancer");
        feederServo2 = hardwareMap.get(CRServo.class, "leftAdvancer");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized. Ready to Shoot First.");
        telemetry.update();

        waitForStart();

        // =======================================================
        // B. REVISED AUTONOMOUS SEQUENCE
        // =======================================================

        // 1. Start the shooter and wait for it to spin up
        telemetry.addData("Path", "1. Spinning up shooter...");
        telemetry.update();
        shooterMotor.setPower(SHOOTER_SPEED);
        sleep(1500); // **Wait 1.5 seconds for the shooter to reach full speed (tune this)**

        // 2. Run the feeders to shoot the game piece(s)
        telemetry.addData("Path", "2. Shooting...");
        telemetry.update();
        feederServo1.setPower(1.0);
        feederServo2.setPower(1.0);
        sleep(1000); // **Run feeders for 1 second (tune this for your number of game pieces)**

        // 3. Stop the feeders and shooter
        telemetry.addData("Path", "3. Stopping mechanisms.");
        telemetry.update();
        feederServo1.setPower(0.0);
        feederServo2.setPower(0.0);
        shooterMotor.setPower(0.0);

        // 4. Drive backward 50 inches to park
        // Note: Negative distance for backward movement.
        telemetry.addData("Path", "4. Driving backward 50 inches.");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, -50.0, 6.0); // Drive -50 inches (backward), 6 second timeout

        telemetry.addData("Status", "Autonomous complete.");
        telemetry.update();
    }

    // =======================================================
    // C. ENCODER DRIVE METHOD (Same as before)
    // =======================================================

    /**
     * Method to perform a precise forward/backward movement using encoders.
     * @param speed The target motor power (0.0 to 1.0).
     * @param inches The distance in inches (+ for forward, - for backward).
     * @param timeoutS The time in seconds before aborting the move.
     */
    public void encoderDrive(double speed, double inches, double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {

            newLeftTarget = leftDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();
            leftDrive.setPower(Math.abs(speed));
            rightDrive.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() && rightDrive.isBusy())) {

                telemetry.addData("Path",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Current", "Running at %7d :%7d",
                        leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
                telemetry.update();
            }

            leftDrive.setPower(0);
            rightDrive.setPower(0);

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}