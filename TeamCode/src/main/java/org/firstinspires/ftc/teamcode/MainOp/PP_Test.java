    package org.firstinspires.ftc.teamcode.MainOp;

    import com.qualcomm.robotcore.eventloop.opmode.OpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    
    import org.firstinspires.ftc.teamcode.Mecanum_Config;
    import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
    
    import java.util.function.Supplier;
    

    @TeleOp(name = "MecanumOG")
    public class PP_Test extends OpMode {

    
        // Toggle states
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
        private final double shooter_Velo = 1750;
    
        private Mecanum_Config Mecanum_Config;
    
    
        @Override
        public void init() {
            Mecanum_Config = new Mecanum_Config(hardwareMap);


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
                intakeOnfwd = !intakeOnfwd;
                if (intakeOnfwd) intakeOnbwd = false; // mutually exclusive
            }
    
            // Intake backward toggle (Y)
            if (gamepad2.yWasPressed()) {
                intakeOnbwd = !intakeOnbwd;
                if (intakeOnbwd) intakeOnfwd = false; // mutually exclusive
            }
    
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
            double intakePower = 0.0;
            if (intakeOnfwd) intakePower = 1.0;
            else if (intakeOnbwd) intakePower = -1.0;
            Mecanum_Config.IntakeMotor.setPower(intakePower);
    
            // Advancers / feeders
            double leftAdvPower = 0.0;
            double rightAdvPower = 0.0;
            if (advancersOnfwd) {
                leftAdvPower = -1.0;
                rightAdvPower = 1.0;
            } else if (advancersOnbwd) {
                leftAdvPower = 1.0;
                rightAdvPower = -1.0;
            }
            Mecanum_Config.leftAdvancer.setPower(leftAdvPower);
            Mecanum_Config.rightAdvancer.setPower(rightAdvPower);
    
            // Shooter (use velocity setting once)
            double shooterVelocity = 0.0;
            if (shooterOnfwd) shooterVelocity = shooter_Velo;
            else if (shooterOnbwd) shooterVelocity = -shooter_Velo;
            // Assuming Mecanum_Config.shooter is a DcMotorEx with setVelocity method
            Mecanum_Config.shooter.setVelocity(shooterVelocity);
    
            // ---------- Status strings (reflect toggles, not instantaneous buttons) ----------
            shooterStatus = shooterOnfwd ? "FORWARD" : shooterOnbwd ? "REVERSE" : "OFF";
            intakeStatus = intakeOnfwd ? "IN" : intakeOnbwd ? "OUT" : "OFF";
            advancerStatus = advancersOnfwd ? "IN" : advancersOnbwd ? "OUT" : "OFF";
    
            telemetry.addData("Status", "Shooter: %s | Intake: %s | Advancers: %s",
                    shooterStatus, intakeStatus, advancerStatus);
            telemetry.update();
    
            // ---------- Automated PathFollowing ----------

        }
    }
