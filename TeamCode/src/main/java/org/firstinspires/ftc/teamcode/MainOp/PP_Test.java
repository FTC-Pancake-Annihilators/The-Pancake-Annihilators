    package org.firstinspires.ftc.teamcode.MainOp;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    
    import org.firstinspires.ftc.teamcode.Mecanum_Config;
    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
    
    import java.util.function.Supplier;
    

    @TeleOp(name = "MecanumOG")
    public class PP_Test extends OpMode {

    
        // Toggle states

    

    
        private Mecanum_Config mecanum;
    
    
        @Override
        public void init() {
            mecanum = new Mecanum_Config(hardwareMap);


            // If you want to set shooter PIDF here, do it in your Mecanum_Config initialization.
            // Example (commented out because Mecanum_Config may not be initialized here):
            // Mecanum_Config.shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        }
    
        @Override
        public void start() {
            // Start teleop drive (follower handles motor control)

        }
    
        @Override
        public void loop() {
            // Update follower and telemetry

    
            // ---------- Driving ----------
            
    
            // ---------- Input toggles (edge triggered) ----------
            // Intake forward toggle (A)
            if (gamepad2.aWasPressed()) {
                mecanum.intakeOnfwd = !mecanum.intakeOnfwd;
                if (mecanum.intakeOnfwd) mecanum.intakeOnbwd = false; // mutually exclusive
            }
    
            // Intake backward toggle (Y)
            if (gamepad2.yWasPressed()) {
                mecanum.intakeOnbwd = !mecanum.intakeOnbwd;
                if (mecanum.intakeOnbwd) mecanum.intakeOnfwd = false; // mutually exclusive
            }
    
            // Advancers forward toggle (X)
            if (gamepad2.xWasPressed()) {
                mecanum.advancersOnfwd = !mecanum.advancersOnfwd;
                if (mecanum.advancersOnfwd) mecanum.advancersOnbwd = false; // mutually exclusive
            }
    
            // Advancers backward toggle (B)
            if (gamepad2.bWasPressed()) {
                mecanum.advancersOnbwd = !mecanum.advancersOnbwd;
                if (mecanum.advancersOnbwd) mecanum.advancersOnfwd = false; // mutually exclusive
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
