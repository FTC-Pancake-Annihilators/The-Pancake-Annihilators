package org.firstinspires.ftc.teamcode.MainOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Config;

@TeleOp(name = "AAAABot_2")
public class Bot_2 extends OpMode {
    public DcMotorEx lf_Drive;
    public DcMotorEx lb_Drive;
    public DcMotorEx rf_Drive;
    public DcMotorEx rb_Drive;

    private Config config;

    public DcMotorEx shooter;
    public CRServo leftAdvancer;
    public CRServo rightAdvancer;
    private boolean intakeOnfwd = false;
    private boolean intakeOnbwd = false;
    private boolean advancersOnfwd = false;
    private boolean advancersOnbwd = false;
    private boolean shooterOnfwd = false;
    private boolean shooterOnbwd = false;

    private String shooterStatus = "OFF";
    private String intakeStatus = "OFF";
    private String advancerStatus = "OFF";

    // Shooter Velo
    private final double shooter_Velo = 2780; // 1750 -> 1850 -> 13000 -> 20000 ->





    @Override
    public void init() {
        lf_Drive = hardwareMap.get(DcMotorEx.class, "lf_Drive");
        rf_Drive = hardwareMap.get(DcMotorEx.class, "rf_Drive");
        lb_Drive = hardwareMap.get(DcMotorEx.class, "lb_Drive");
        rb_Drive = hardwareMap.get(DcMotorEx.class, "rb_Drive");

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        leftAdvancer = hardwareMap.get(CRServo.class, "leftAdvancer");
        rightAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");
        lf_Drive.setDirection(DcMotorEx.Direction.REVERSE);
        lb_Drive.setDirection(DcMotorEx.Direction.REVERSE);
        rf_Drive.setDirection(DcMotorEx.Direction.FORWARD);
        rb_Drive.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        lf_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb_Drive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lb_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lf_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rb_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rf_Drive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));



        lf_Drive.setDirection(DcMotor.Direction.REVERSE);
        rf_Drive.setDirection(DcMotor.Direction.FORWARD);
        lf_Drive.setDirection(DcMotor.Direction.REVERSE);
        rb_Drive.setDirection(DcMotor.Direction.FORWARD);

        lf_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb_Drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);


        // Advancers forward toggle (X)
        if (gamepad2.xWasPressed()) {
            advancersOnfwd = !advancersOnfwd;
            if (advancersOnfwd) advancersOnbwd = false; // mutually exclusive
        }

        // Advancers backward toggle (B)
        if (gamepad2.bWasPressed()) {
            advancersOnbwd = !advancersOnbwd;
            if (advancersOnbwd) advancersOnfwd = false; // mutually exclusive
        }



//
        // Shooter forward toggle (right bumper)
        if (gamepad2.rightBumperWasPressed()) {
            shooterOnfwd = !shooterOnfwd;
            if (shooterOnfwd) shooterOnbwd = false; // mutually exclusive
        }

        // Shooter reverse toggle (left bumper)
        if (gamepad2.leftBumperWasPressed()) {
            shooterOnbwd = !shooterOnbwd;
            if (shooterOnbwd) shooterOnfwd = false; // mutually exclusive
        }

        // ---------- Apply motor outputs (set each motor once) ----------
        // Intake motor


        // Advancers / feeders
        double leftAdvPower = 0.0;
        double rightAdvPower = 0.0;
        if (advancersOnfwd) {
            leftAdvPower = 1.0;
            rightAdvPower = 1.0;
        } else if (advancersOnbwd) {
            leftAdvPower = -1.0;
            rightAdvPower = -1.0;
        }
        leftAdvancer.setPower(leftAdvPower);
        rightAdvancer.setPower(rightAdvPower);

        // Shooter (use velocity setting once)
        double shooterVelocity = 0.0;
        if (shooterOnfwd) shooterVelocity = shooter_Velo;
        else if (shooterOnbwd) shooterVelocity = -shooter_Velo;
        // Assuming Mecanum_Config.shooter is a DcMotorEx with setVelocity method
        shooter.setVelocity(shooterVelocity);

        // ---------- Status strings (reflect toggles, not instantaneous buttons) ----------
        shooterStatus = shooterOnfwd ? "FORWARD" : shooterOnbwd ? "REVERSE" : "OFF";
       // intakeStatus = intakeOnfwd ? "IN" : intakeOnbwd ? "OUT" : "OFF";
        advancerStatus = advancersOnfwd ? "IN" : advancersOnbwd ? "OUT" : "OFF";
        telemetry.addData("Actual DPS", shooter.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("Actual TPS", shooter.getVelocity());
        telemetry.update();
        telemetry.addData("Status", "Shooter: %s | Intake: %s | Advancers: %s",
                shooterStatus, intakeStatus, advancerStatus);
        telemetry.update();

        // ---------- Automated PathFollowing ----------

    }
    void mecanumDrive(double forward, double strafe, double rotate){

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        double leftFrontPower = (forward + strafe + rotate) / denominator;
        double rightFrontPower = (forward - strafe - rotate) / denominator;
        double leftBackPower = (forward - strafe + rotate) / denominator;
        double rightBackPower = (forward + strafe - rotate) / denominator;

        lf_Drive.setPower(leftFrontPower);
        rf_Drive.setPower(rightFrontPower);
        lb_Drive.setPower(leftBackPower);
        rb_Drive.setPower(rightBackPower);
    }

}
