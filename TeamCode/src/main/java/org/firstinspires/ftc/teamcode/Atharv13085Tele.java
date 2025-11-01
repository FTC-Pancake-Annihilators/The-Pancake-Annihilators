package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx.ZeroPowerBehavior;

import com.qualcomm.robotcore.util.Range;

// TODO: Use dtState enum to clarify drivetrain speed and remove redundant gamepad1.left_trigger fetches.
/**
 * @author Atharv G - 13085 Bionic Dutch
 *
 * Teaches Hrithik how to use an enumerator
 */
@TeleOp(name="AtharvTele")
public class Atharv13085Tele extends LinearOpMode {

    private enum shooterState {
        OFF, NEAR, FAR
    }
    private enum intakeState {
        OFF, IN, OUT
    }
    private enum dtState {
        SLOW, NORMAL
    }

    private DcMotorEx leftMotor, rightMotor;
    private DcMotorEx intakeMotor;
    private CRServo leftAdvancer, rightAdvancer;
    private DcMotorEx shooter;
    private shooterState shooterStatus;
    private intakeState intakeStatus;
    private intakeState advancerStatus;

    private final double FAR_POWER  = 0.75;
    private final double NEAR_POWER = 0.4;
    private double drive, turn, speedScale;
    private double shooterPower;

    private boolean leftTrigger;


    @Override
    public void runOpMode() {

        // DRIVE MOTORS
        leftMotor  = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");

        leftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        // INTAKE + ADVANCERS
        intakeMotor   = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftAdvancer  = hardwareMap.get(CRServo.class, "leftAdvancer");
        rightAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");

        // SHOOTER
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        //  HELPER VARIABLES
        speedScale = 1.0;

        shooterStatus = shooterState.OFF;
        intakeStatus = intakeState.OFF;
        advancerStatus = intakeState.OFF;

        telemetry.addLine("Initialized ✅");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            leftTrigger = gamepad1.left_trigger > 0.1;

            // --- DRIVE (Gamepad1) ---
            drive = -gamepad1.left_stick_y;
            turn  = gamepad1.right_stick_x;

            // Determine speed mode
            if (leftTrigger)        speedScale = 0.5;  // slow
            else if (gamepad1.right_trigger > 0.1)  speedScale = 1.0; // normal (override slow)
            else                                    speedScale = 1.0;

            //  Calculate and set power to motors
            double leftPower  = Range.clip((drive * speedScale) - (turn * speedScale), -1, 1);
            double rightPower = Range.clip((drive * speedScale) + (turn * speedScale), -1, 1);

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
            switch (advancerStatus) {
                case intakeState.OFF:
                    leftAdvancer.setPower(0);
                    rightAdvancer.setPower(0);
                case intakeState.IN:
                    leftAdvancer.setPower(-1);
                    rightAdvancer.setPower(1);
            }
            if (gamepad2.x) {
                advancerStatus = intakeState.IN;
            } else {
                advancerStatus = intakeState.OFF;
            }

            // --- SHOOTER (Gamepad2) ---
            switch (shooterStatus) {
                case shooterState.OFF:
                    shooter.setPower(0);
                case shooterState.FAR:
                    shooter.setPower(FAR_POWER);
                case shooterState.NEAR:
                    shooter.setPower(NEAR_POWER);
            }

            if (gamepad2.right_bumper) {
                shooterStatus = shooterState.FAR;
            }
            else if (gamepad2.left_bumper) {
                shooterStatus = shooterState.NEAR;
            }
            else {
                shooterStatus = shooterState.OFF;
            }

            // --- TELEMETRY ---
            // Reassign values (no redeclaration)

            telemetry.addData("Status", "Shooter: %s | Intake: %s | Advancers: %s",
                                        shooterStatus, intakeStatus, advancerStatus);

            telemetry.addData("Drive L/R", "L: %.2f R: %.2f",
                                            leftPower, rightPower);

            telemetry.addData("Drive Mode", leftTrigger ? "SLOW (0.5)" : "NORMAL (1.0)");
            telemetry.update();
        }
    }
}
//// Hey guys! This is your friendly neighborhood software dev
//// from team 13085!
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
//
//import com.qualcomm.robotcore.util.Range;
//
//@TeleOp(name="TeleOp_Final_NoRumble", group="TE LEOP")
//public class TeleOp_Final extends LinearOpMode {
//
//    private enum shooterState {
//        OFF, NEAR, FAR
//    }
//    private enum intakeState {
//        OFF, IN, OUT
//    }
//
//    private DcMotor leftMotor, rightMotor;
//    private DcMotor intakeMotor;
//    private CRServo leftAdvancer, rightAdvancer;
//    private DcMotorEx shooter;
//    private shooterState shooterStatus;
//    private intakeState intakeStatus;
//    private intakeState advancerStatus;
//
//    private final double FAR_POWER  = 0.75;
//    private final double NEAR_POWER = 0.4;
//
//
//    @Override
//    public void runOpMode() {
//
//        // DRIVE MOTORS
//        leftMotor  = hardwareMap.get(DcMotor.class, "leftMotor");
//        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
//
//        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
//        rightMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
//
//        // INTAKE + ADVANCERS
//        intakeMotor   = hardwareMap.get(DcMotor.class, "intakeMotor");
//        leftAdvancer  = hardwareMap.get(CRServo.class, "leftAdvancer");
//        rightAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");
//
//        // SHOOTER
//        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//
//        telemetry.addLine("Initialized ✅");
//        telemetry.update();
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            // --- DRIVE (Gamepad1) ---
//            double drive = -gamepad1.left_stick_y;
//            double turn  = gamepad1.right_stick_x;
//
//            // Determine speed mode
//            double speedScale = 1.0; // default full speed
//            if (gamepad1.left_trigger > 0.2) speedScale = 0.5;  // slow
//            if (gamepad1.right_trigger > 0.2) speedScale = 1.0; // normal (override slow)
//
//            double leftPower  = Range.clip((drive*speedScale) - (turn*speedScale), -1, 1);
//            double rightPower = Range.clip((drive*speedScale) + (turn*speedScale), -1, 1);
//
//            leftMotor.setPower(leftPower);
//            rightMotor.setPower(rightPower);
//
//            // --- INTAKE (Gamepad2) ---
//            if (gamepad2.y) {
//                intakeMotor.setPower(1); // intake in
//            } else if (gamepad2.a) {
//                intakeMotor.setPower(-1); // intake out
//            } else {
//                intakeMotor.setPower(0);  // stop
//            }
//
//            // --- ADVANCERS (Gamepad2) ---
//            if (gamepad2.x) {
//                leftAdvancer.setPower(-1);
//                rightAdvancer.setPower(1);
//            } else {
//                leftAdvancer.setPower(0);
//                rightAdvancer.setPower(0);
//            }
//
//            // --- SHOOTER (Gamepad2) ---
//            switch (shooterStatus) {
//                case shooterState.OFF:
//                    shooter.setPower(0);
//                case shooterState.FAR:
//                    shooter.setPower(FAR_POWER);
//                case shooterState.NEAR:
//                    shooter.setPower(NEAR_POWER);
//            }
//
//            if (gamepad2.right_bumper) {
//                shooterStatus = shooterState.FAR;
//            }
//            else if (gamepad2.left_bumper) {
//                shooterStatus = shooterState.NEAR;
//            }
//            else {
//                shooterStatus = shooterState.OFF;
//            }
//
//            // --- TELEMETRY ---
//            // Reassign values (no redeclaration)
//
//            telemetry.addData("Status", "Shooter: %s | Intake: %s | Advancers: %s",
//                    shooterStatus, intakeStatus, advancerStatus);
//            telemetry.addData("Drive L/R", "L: %.2f R: %.2f", leftPower, rightPower);
//            telemetry.addData("Drive Mode", gamepad1.left_trigger > 0.2 ? "SLOW (0.5)" : "NORMAL (1.0)");
//            telemetry.update();
//        }
//    }
//}