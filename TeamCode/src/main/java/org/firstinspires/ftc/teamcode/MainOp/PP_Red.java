package org.firstinspires.ftc.teamcode.MainOp;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.Mecanum_Config;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;
@Configurable
@TeleOp(name = "MecanumOG")
public class PP_Red extends OpMode {
    private Mecanum_Config Mecanum_Config;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    //Intake, Feeders, Shooters
    boolean intakeOnfwd, intakeOnbwd, advancersOnfwd, advancersOnbwd, shooterOnfwd, shooterOnbwd;

    private String shooterStatus = "OFF";
    private String intakeStatus = "OFF";
    private String advancerStatus = "OFF";


    //Shooter Velo

    private final double shooter_Velo = 1750;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(96.48, 95.6))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
       // Mecanum_Config.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
    }
    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }
    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
             follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
             // Intake Forward



            if (gamepad2.aWasPressed() && !intakeOnfwd) {
                intakeOnfwd = true;


            }
            if (gamepad2.aWasPressed() && intakeOnfwd) {
                intakeOnfwd = false;
            }
            if (intakeOnfwd) {
                Mecanum_Config.IntakeMotor.setPower(1);

            } else {
                Mecanum_Config.IntakeMotor.setPower(0);

            }
            //Intake reverse



            if (gamepad2.yWasPressed() && !intakeOnbwd) {
                intakeOnbwd = true;


            }
            if (gamepad2.yWasPressed() && intakeOnbwd) {
                intakeOnbwd = false;
            }
            if (intakeOnbwd) {
                Mecanum_Config.IntakeMotor.setPower(-1);

            } else {
                Mecanum_Config.IntakeMotor.setPower(0);

            }



            //Feeders Forward
            if (gamepad2.xWasPressed() && !advancersOnfwd) {
                advancersOnfwd = true;


            }
            if (gamepad2.xWasPressed() && advancersOnfwd) {
                advancersOnfwd = false;
            }
            if (advancersOnfwd) {
                Mecanum_Config.leftAdvancer.setPower(-1);
                Mecanum_Config.rightAdvancer.setPower(1);

            } else {
                Mecanum_Config.leftAdvancer.setPower(0);
                Mecanum_Config.rightAdvancer.setPower(0);

            }

            //Feeders Backward
            if (gamepad2.bWasPressed() && !advancersOnbwd) {
                advancersOnbwd = true;


            }
            if (gamepad2.bWasPressed() && advancersOnbwd) {
                advancersOnbwd = false;
            }
            if (advancersOnbwd) {
                Mecanum_Config.leftAdvancer.setPower(1);
                Mecanum_Config.rightAdvancer.setPower(-1);

            } else {
                Mecanum_Config.leftAdvancer.setPower(0);
                Mecanum_Config.rightAdvancer.setPower(0);

            }

        }

        // Shooter Forward



        if (gamepad2.rightBumperWasPressed() && !shooterOnfwd) {
            shooterOnfwd = true;


        }
        if (gamepad2.rightBumperWasPressed() && shooterOnfwd) {
            shooterOnfwd = false;
        }
        if (shooterOnfwd) {
            Mecanum_Config.shooter.setVelocity(shooter_Velo);

        } else {
            Mecanum_Config.shooter.setVelocity(0);

        }
        //shooter reverse



        if (gamepad2.leftBumperWasPressed() && !shooterOnbwd) {
            shooterOnbwd = true;


        }
        if (gamepad2.leftBumperWasPressed() && shooterOnbwd) {
            shooterOnbwd = false;
        }
        if (shooterOnbwd) {
            Mecanum_Config.shooter.setVelocity(-shooter_Velo);

        } else {
            Mecanum_Config.shooter.setVelocity(0);

        }


        shooterStatus  = gamepad2.right_bumper ? "FORWARD" : gamepad2.left_bumper ? "REVERSE" : "OFF";
        intakeStatus   = gamepad2.a ? "IN" : gamepad2.y ? "OUT" : "OFF";
        advancerStatus = gamepad2.x ? "IN" :gamepad2.b ? "OUT" : "OFF";

        telemetry.addData("Status", "Shooter: %s | Intake: %s | Advancers: %s",
                shooterStatus, intakeStatus, advancerStatus);
        telemetry.update();









        //Automated PathFollowing
        if (gamepad1.rightBumperWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.leftBumperWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}
