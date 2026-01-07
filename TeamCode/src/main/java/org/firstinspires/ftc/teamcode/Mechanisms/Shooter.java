package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.control.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.control.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.AllianceColor;

public class Shooter {
    private PIDFController shooterPidf;
    private DcMotorEx shooterMotor;

    // Alliance multiplier (tune if one side shoots differently)
    private final double RED_MULTIPLIER = 1.1;
    private final double BLUE_MULTIPLIER = 1.1;

    // Safe velocity limits (degrees per second - tune on practice field!)
    private static final double MIN_VELOCITY = 140.0;  // For close shots
    private static final double MAX_VELOCITY = 320.0;  // For far shots

    public Shooter(HardwareMap hwMap, PIDFCoefficients shooterCoefficients) {
        shooterMotor = hwMap.get(DcMotorEx.class, "shooter");
        shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Correct: method calls with ()
        shooterMotor.setVelocityPIDFCoefficients(
                300,0,0,10
        );

        shooterPidf = new PIDFController(shooterCoefficients);
    }

    // Reverse pulse (only power used - brief)
    public void eject() {
        shooterMotor.setPower(-0.1);
    }

    // Auto velocity from distance
    public void adaptive(double robotX, double robotY, AllianceColor alliance) {
        double distance = getDistanceToGoal(robotX, robotY, alliance);
        double targetVelocity = getRegressionVelocity(distance, alliance);
        setTargetVelocity(targetVelocity);
    }

    public void stop() {
        setTargetVelocity(0.0);
    }

    public void idle() {
        setTargetVelocity(50.0);  // Low spin
    }

    public void midFieldShoot() {
        setTargetVelocity(Constants.midFieldVelocity);
    }

    public void farShoot() {
        setTargetVelocity(Constants.farVelocity);
    }

    // Set velocity with PIDF
    private void setTargetVelocity(double targetVelocity) {
        shooterPidf.updatePosition(shooterMotor.getVelocity(AngleUnit.DEGREES));
        shooterPidf.setTargetPosition(targetVelocity);
        double pidPower = shooterPidf.run();
        shooterMotor.setPower(MathFunctions.clamp(pidPower, -1.0, 1.0));
    }

    public double getTargetVelocity() {
        return shooterPidf.getTargetPosition();
    }

    private double getDistanceToGoal(double robotX, double robotY, AllianceColor alliance) {
        if (alliance.isRed()) {
            return Math.hypot(144 - robotX, 144 - robotY);
        } else {
            return Math.hypot(robotX, 144 - robotY);  // Blue goal at (0,144)
        }
    }

    private double getRegressionVelocity(double distance, AllianceColor alliance) {
        double baseVelocity = 1.10033 * distance + 93.21422;
        double adjustedVelocity = baseVelocity * (alliance.isRed() ? RED_MULTIPLIER : BLUE_MULTIPLIER);
        return Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, adjustedVelocity));
    }
}