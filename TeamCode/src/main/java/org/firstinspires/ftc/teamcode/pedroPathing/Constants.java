package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.0)
            .forwardZeroPowerAcceleration(-34.0)
            .lateralZeroPowerAcceleration(-59.0)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.30, 0.00001, 0.04, 0.020))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.22, 0.0001, 0.02, 0.018))
            .useSecondaryTranslationalPIDF(true)

            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0.001, 0.1, 0.025))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.5, 0.005, 0.03, 0.03))
            .useSecondaryHeadingPIDF(true)

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0.00001, 0.007, 0.6, 0.02))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.002, 0.00001, 0.0001, 0.6, 0.05))
            .useSecondaryDrivePIDF(true)


            .automaticHoldEnd(true);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf_Drive")
            .rightRearMotorName("rb_Drive")
            .leftRearMotorName("lb_Drive")
            .leftFrontMotorName("lf_Drive")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(67.0)
            .yVelocity(56)
            .useBrakeModeInTeleOp(false);



    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("rf_Drive")
            .rightRearMotorName("rb_Drive")
            .leftRearMotorName("lb_Drive")
            .leftFrontMotorName("lf_Drive")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD);
//            .forwardTicksToInches(multiplier)
//            .strafeTicksToInches(multiplier)
//            .turnTicksToInches(multiplier);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.01,
            0.01,
            0.001,
            50,
            1.8,
            10,
            0.9);



    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                // Use one of the localizers below:
                .driveEncoderLocalizer(localizerConstants)

                .build();
    }
}
